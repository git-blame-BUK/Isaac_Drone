#!/usr/bin/env python3
# uav_3d_trajectory_planner_node.py

from __future__ import annotations

from typing import Tuple, Optional, List

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# nvblox ESDF service (check message fields with `ros2 interface show`)
from nvblox_msgs.srv import EsdfAndGradients

from Planner_core import (
    VoxelGridInfo,
    VoxelGrid,
    esdf_to_cost_grid,
    AStar3DPlanner,
)


class Uav3DTrajectoryPlannerNode(Node):
    """
    ROS2 node that:

      1. Subscribes to:
         - current UAV pose (/mavros/local_position/pose)
         - goal pose (/uav/goal_pose)

      2. Calls the nvblox ESDF service
         (/nvblox_node/get_esdf_and_gradient) to obtain a 3D ESDF grid
         in a local axis-aligned bounding box (AABB) around start & goal.

      3. Converts ESDF -> cost grid (using esdf_to_cost_grid).

      4. Runs A* (AStar3DPlanner) to compute a safe 3D path.

      5. Publishes the path as nav_msgs/Path on /uav/trajectory.
         Your offboard controller can then follow this path.
    """

    def __init__(self):
        super().__init__("uav_3d_trajectory_planner")

        # --- Subscriptions: current pose & goal pose ---

        self.current_pose: Optional[PoseStamped] = None
        self.current_goal: Optional[PoseStamped] = None

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.current_pose_callback,
            10,
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "/uav/goal_pose",
            self.goal_pose_callback,
            10,
        )

        # --- Publisher: planned trajectory ---

        self.trajectory_publisher = self.create_publisher(
            Path,
            "/uav/trajectory",
            10,
        )

        # --- nvblox ESDF service client ---

        self.esdf_service_client = self.create_client(
            EsdfAndGradients,
            "/nvblox_node/get_esdf_and_gradient",
        )

        # --- Parameters for planning ---

        # How far around start & goal we ask nvblox for ESDF data (meters)
        self.declare_parameter("bbox_xy_margin", 10.0)         # horizontal margin
        self.declare_parameter("bbox_z_margin", 5.0)           # vertical margin

        # Resolution of ESDF grid we request from nvblox (meters per voxel)
        self.declare_parameter("voxel_size", 0.2)

        # ESDF -> cost conversion
        self.declare_parameter("safety_radius", 0.4)           # min allowed distance to obstacles
        self.declare_parameter("inflation_radius", 2.0)        # where obstacle influence fades out
        self.declare_parameter("free_cost", 1.0)               # cost in free space
        self.declare_parameter("max_cost", 5.0)                # cost near safety boundary

        self.get_logger().info("Uav3DTrajectoryPlannerNode started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def current_pose_callback(self, msg: PoseStamped) -> None:
        """
        Store the latest UAV pose.
        """
        self.current_pose = msg

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Store the latest goal pose and trigger planning.
        """
        self.current_goal = msg
        self.get_logger().info("New goal received, starting 3D planning...")
        self.plan_and_publish_trajectory()

    # ------------------------------------------------------------------
    # Main planning pipeline
    # ------------------------------------------------------------------

    def plan_and_publish_trajectory(self) -> None:
        """
        Complete planning pipeline:
          - check if we have pose & goal
          - compute AABB around start & goal
          - query nvblox ESDF in that AABB
          - convert ESDF -> cost grid
          - run A* to plan 3D path
          - publish nav_msgs/Path
        """
        if self.current_pose is None or self.current_goal is None:
            self.get_logger().warn("Cannot plan: missing current pose or goal pose.")
            return

        start_position = self.current_pose.pose.position
        goal_position = self.current_goal.pose.position

        bbox_xy_margin = float(self.get_parameter("bbox_xy_margin").value)
        bbox_z_margin = float(self.get_parameter("bbox_z_margin").value)
        voxel_size = float(self.get_parameter("voxel_size").value)
        safety_radius = float(self.get_parameter("safety_radius").value)
        inflation_radius = float(self.get_parameter("inflation_radius").value)
        free_cost = float(self.get_parameter("free_cost").value)
        max_cost = float(self.get_parameter("max_cost").value)

        # 1) Compute local bounding box (AABB) around start & goal
        min_x = min(start_position.x, goal_position.x) - bbox_xy_margin
        max_x = max(start_position.x, goal_position.x) + bbox_xy_margin
        min_y = min(start_position.y, goal_position.y) - bbox_xy_margin
        max_y = max(start_position.y, goal_position.y) + bbox_xy_margin
        min_z = min(start_position.z, goal_position.z) - bbox_z_margin
        max_z = max(start_position.z, goal_position.z) + bbox_z_margin

        # 2) Request ESDF from nvblox inside this AABB
        esdf_query_result = self.query_esdf_for_aabb(
            aabb_min=(min_x, min_y, min_z),
            aabb_max=(max_x, max_y, max_z),
            voxel_size=voxel_size,
        )

        if esdf_query_result is None:
            self.get_logger().warn("ESDF service returned no data. Planning aborted.")
            return

        esdf_grid, voxel_grid_info = esdf_query_result  # esdf_grid: shape (Z, Y, X)

        # 3) Convert ESDF to cost grid (safe distance + inflation costs)
        cost_grid = esdf_to_cost_grid(
            esdf_grid,
            safety_radius=safety_radius,
            inflation_radius=inflation_radius,
            free_cost=free_cost,
            max_cost=max_cost,
        )

        # 4) Run 3D A* planner on the cost grid
        voxel_grid = VoxelGrid(voxel_grid_info)
        planner = AStar3DPlanner(voxel_grid, cost_grid)

        start_world = (start_position.x, start_position.y, start_position.z)
        goal_world = (goal_position.x, goal_position.y, goal_position.z)

        path_world_points: List[Tuple[float, float, float]] = planner.plan_path(
            start_world,
            goal_world,
        )

        if not path_world_points:
            self.get_logger().warn("No 3D path found.")
            return

        # 5) Convert the path into a nav_msgs/Path and publish
        path_message = self.create_path_message(path_world_points,
                                                self.current_pose.header.frame_id)
        self.trajectory_publisher.publish(path_message)
        self.get_logger().info(f"Published 3D trajectory with {len(path_world_points)} points.")

    # ------------------------------------------------------------------
    # ESDF service interaction with nvblox
    # ------------------------------------------------------------------

    def query_esdf_for_aabb(
        self,
        aabb_min: Tuple[float, float, float],
        aabb_max: Tuple[float, float, float],
        voxel_size: float,
    ) -> Optional[Tuple[np.ndarray, VoxelGridInfo]]:
        """
        Call the nvblox ESDF service /nvblox_node/get_esdf_and_gradient
        for a given axis-aligned bounding box (AABB).

        Returns:
            (esdf_grid, voxel_grid_info) if successful, else None.

        IMPORTANT:
          You MUST check the exact fields of EsdfAndGradients.srv in
          your workspace with:

            ros2 interface show nvblox_msgs/srv/EsdfAndGradients

          and adapt the field names below accordingly.
        """
        if not self.esdf_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("ESDF service not available.")
            return None

        request = EsdfAndGradients.Request()

        # TODO: adapt these fields to the actual service definition!
        try:
            # Typical pattern (verify with `ros2 interface show`):
            # request.aabb_min.x/y/z, request.aabb_max.x/y/z, request.voxel_size, request.frame_id, etc.

            request.aabb_min.x = float(aabb_min[0])
            request.aabb_min.y = float(aabb_min[1])
            request.aabb_min.z = float(aabb_min[2])

            request.aabb_max.x = float(aabb_max[0])
            request.aabb_max.y = float(aabb_max[1])
            request.aabb_max.z = float(aabb_max[2])

            request.voxel_size = float(voxel_size)
            # If present in your service:
            # request.frame_id = "map"
        except AttributeError:
            self.get_logger().error(
                "Please adapt request field names to match EsdfAndGradients.srv "
                "(e.g. aabb_min, aabb_max, voxel_size, frame_id, ...)."
            )
            return None

        future = self.esdf_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().warn("ESDF service call failed.")
            return None

        response = future.result()

        # === Convert response to numpy ESDF grid + VoxelGridInfo ===
        try:
            # Typical pattern (verify with `ros2 interface show`):
            # response.dim_x, response.dim_y, response.dim_z
            # response.esdf (flattened float array)
            # response.origin.x/y/z

            dim_x = response.dim_x
            dim_y = response.dim_y
            dim_z = response.dim_z

            esdf_flat = np.array(response.esdf, dtype=np.float32)
            esdf_grid = esdf_flat.reshape(dim_z, dim_y, dim_x)

            origin_x = response.origin.x
            origin_y = response.origin.y
            origin_z = response.origin.z

        except AttributeError:
            self.get_logger().error(
                "Please adapt response field names (dim_x/y/z, esdf, origin) "
                "to match EsdfAndGradients.srv."
            )
            return None

        voxel_grid_info = VoxelGridInfo(
            resolution=voxel_size,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_z=origin_z,
            size_x=dim_x,
            size_y=dim_y,
            size_z=dim_z,
        )

        return esdf_grid, voxel_grid_info

    # ------------------------------------------------------------------
    # Helper to build nav_msgs/Path
    # ------------------------------------------------------------------

    def create_path_message(
        self,
        path_world_points: List[Tuple[float, float, float]],
        frame_id: str,
    ) -> Path:
        """
        Convert a list of world points (x,y,z) into a nav_msgs/Path.
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for x, y, z in path_world_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            # Simple orientation (no yaw control yet)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = Uav3DTrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

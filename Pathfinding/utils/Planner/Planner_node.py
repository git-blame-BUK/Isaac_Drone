#!/usr/bin/env python3
# uav_3d_trajectory_planner_node.py
# has to be started inside ov nvblox dev container depends on nvblox ROS2 package nvblox_msgs

from __future__ import annotations

from typing import Tuple, Optional, List
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


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

        # --- Callback group for service client (prevents deadlock) ---
        # Service responses can be processed concurrently with other callbacks.
        self.esdf_cb_group = ReentrantCallbackGroup()

        # --- Subscriptions: current pose & goal pose ---

        self.current_pose: Optional[PoseStamped] = None
        self.current_goal: Optional[PoseStamped] = None

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.current_pose_callback,
            qos_profile_sensor_data,
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
            callback_group=self.esdf_cb_group,
        )

        # --- Parameters for planning ---

        # How far around start & goal we ask nvblox for ESDF data (meters)
        self.declare_parameter("bbox_xy_margin", 2.0)         # horizontal margin
        self.declare_parameter("bbox_z_margin", 1.5)           # vertical margin

        # Resolution of ESDF grid we request from nvblox (meters per voxel)
        self.declare_parameter("voxel_size", 0.2)
        #timeout for esdf service call
        self.declare_parameter("esdf_timeout_sec", 2.0)

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
    def query_esdf_for_aabb(self, aabb_min, aabb_max, voxel_size=None, frame_id="odom"):
        """
        Query nvblox ESDF+gradients within an AABB.

        Parameters
        ----------
        aabb_min : array-like (x,y,z)
        aabb_max : array-like (x,y,z)
        voxel_size : unused (nvblox defines voxel size internally, returned in response.voxel_size_m)
        frame_id : str, frame for AABB (must match nvblox frame)

        Returns
        -------
        (esdf, voxel_grid_info) or None
        esdf: np.ndarray shape (z,y,x) float32
        voxel_grid_info: VoxelGridInfo (resolution, origin, sizes)
        """
        try:
            from nvblox_msgs.srv import EsdfAndGradients
        except Exception as e:
            self.get_logger().error(f"Failed to import nvblox_msgs.srv.EsdfAndGradients: {e}")
            return None

        if not self.esdf_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("ESDF service not available yet: /nvblox_node/get_esdf_and_gradient")
            return None

        # ---- Build request (matches `ros2 interface show nvblox_msgs/srv/EsdfAndGradients`) ----
        req = EsdfAndGradients.Request()
        req.update_esdf = True
        req.visualize_esdf = False
        req.use_aabb = True
        req.frame_id = str(frame_id)

        req.aabb_min_m.x = float(aabb_min[0])
        req.aabb_min_m.y = float(aabb_min[1])
        req.aabb_min_m.z = float(aabb_min[2])

        size_x = float(aabb_max[0] - aabb_min[0])
        size_y = float(aabb_max[1] - aabb_min[1])
        size_z = float(aabb_max[2] - aabb_min[2])

        # Guard against negative/zero boxes (can happen if min/max swapped)
        if size_x <= 0.0 or size_y <= 0.0 or size_z <= 0.0:
            self.get_logger().error(
                f"Invalid AABB: min={aabb_min}, max={aabb_max} -> size=({size_x},{size_y},{size_z}). "
                "Ensure aabb_max > aabb_min in all axes."
            )
            return None

        req.aabb_size_m.x = size_x
        req.aabb_size_m.y = size_y
        req.aabb_size_m.z = size_z

        # ---- Call service with timeout (prevents silent hanging) ----
        timeout_sec = float(self.get_parameter("esdf_timeout_sec").value)

        future = self.esdf_service_client.call_async(req)
        ok = rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(f"ESDF service timed out after {timeout_sec:.2f}s (no response).")
            return None

        res = future.result()

        if res is None:
            self.get_logger().error("ESDF service call returned None (future.result() is None).")
            return None
        if not getattr(res, "success", False):
            self.get_logger().warn("ESDF service call failed: response.success == False")
            return None

        # ---- Parse response ----
        origin_x = float(res.origin_m.x)
        origin_y = float(res.origin_m.y)
        origin_z = float(res.origin_m.z)
        voxel = float(res.voxel_size_m)

        msg = res.esdf_and_gradients
        data = np.asarray(msg.data, dtype=np.float32)

        dims = [int(d.size) for d in msg.layout.dim]
        if len(dims) == 3:
            # (z, y, x)
            z, y, x = dims
            channels = 1
        elif len(dims) == 4:
            # (z, y, x, channels)
            z, y, x, channels = dims
        else:
            self.get_logger().error(f"Unexpected esdf_and_gradients layout dims: {dims}")
            return None

        expected_len = z * y * x * channels
        if data.size != expected_len:
            self.get_logger().warn(
                f"MultiArray data length mismatch: got {data.size}, expected {expected_len} "
                f"for dims={dims}. Trying best-effort reshape..."
            )
            # best-effort: infer from data length if possible
            if channels > 0 and data.size == z * y * x:
                channels = 1
            else:
                return None

        if channels == 1:
            esdf = data.reshape((z, y, x))
        else:
            esdf_grad = data.reshape((z, y, x, channels))
            esdf = esdf_grad[:, :, :, 0]  # channel 0 = ESDF

        # optional debug
        self.get_logger().info(
            f"ESDF received: dims={dims}, voxel={voxel:.4f}m, origin=({origin_x:.2f},{origin_y:.2f},{origin_z:.2f})"
        )

        voxel_grid_info = VoxelGridInfo(
            resolution=voxel,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_z=origin_z,
            size_x=x,
            size_y=y,
            size_z=z,
        )

        return esdf, voxel_grid_info


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

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

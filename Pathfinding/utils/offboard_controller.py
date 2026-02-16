#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import math
from nav_msgs.msg import Path

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class OffboardController(Node):
    """
    Node to control a drone in OFFBOARD mode using MAVROS.
    Subscribes to /mavros/state for FCU state, /mavros
    /local_position/pose for current local position, and /uav/trajectory for planned path.
    Publishes position setpoints to /mavros/setpoint_position/local.
    Implements a simple state machine to manage takeoff, trajectory following, and holding.
    20 Hz update rate.
    TODO : implement landing method in "done" phase.
    TODO : implement safety checks for altitude and position limits.
    TODO : implement smoother trajectory following (interpolation, velocity control).
    """

    def __init__(self):
        super().__init__('offboard_controller')

        # === Subscriptions ===
        # FCU state (connection, armed flag, current mode)
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        # Local position of the drone
        # Note: QoS should match MAVROS (often BEST_EFFORT) to avoid reliability warnings.
        qos_pose = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos_pose,
        )

        # Planned path from the planner
        self.traj_sub = self.create_subscription(
            Path,
            '/uav/trajectory',
            self.trajectory_cb,
            10
        )

        # === Publishers ===
        # Offboard position setpoints (ENU frame from MAVROS)
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # === Service clients ===
        # Arming service client (unused; arming is manual)
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        # Flight mode change service client (unused; mode set manually)
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )
        

        # Internal state
        self.current_state: State | None = None
        self.current_pose: PoseStamped | None = None
        self.ref_pose: PoseStamped | None = None  # start pose (reference)
        self.yaw_cmd = None


        # Mode gate helpers
        self.last_mode = None
        self.last_connected = None
        self.last_state_time = None  # time of last /mavros/state message

        # Trajectory buffer
        self.traj_points: list[tuple[float, float, float]] = []
        self.trja_frame_id: str | None = None
        self.traj_index: int = 0 # to track progress along trajectory
        self.traj_sequence: int = 0 # to track new trajectories

        #Holding Point -> last commanded setpoint 
        self.hold_point: tuple[float, float, float] | None = None

        # Parameters
        self.declare_parameter('takeoff_altitude', 1.0)  # 50 cm hovering alt
        self.declare_parameter('goal_tolerance', 0.10)  # tolerance when goal reached
        self.declare_parameter('min_setpoint_rate', 10.0)  # Hz
        # Simple sequence phases:
        # None -> wait_offboard -> taking_off ->holding -> following_trajecory -> holding
        self.phase: str | None = None

        # Timer for periodic logic (20 Hz)
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("OffboardController initialized. Waiting for FCU state and pose...")

    # === Callbacks ===
    def state_cb(self, msg: State):
        """Cache FCU state and update heartbeat time."""
        self.current_state = msg
        self.last_state_time = self.get_clock().now()

    def pose_cb(self, msg: PoseStamped):
        """Cache local pose."""
        self.current_pose = msg
        # yaw initialization
        if self.yaw_cmd is None:
            self.yaw_cmd = self.yaw_from_quat(msg.pose.orientation)
            self.get_logger().info(f"[INFO] yaw_cmd initialized to {self.yaw_cmd:.3f} rad")

    def trajectory_cb(self, msg: Path):
        # cache new trajectory points
        if not msg.poses:
            self.get_logger().warn("[WARN] received empty trajectory")
            return
        pts = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            pts.append((float(p.x), float(p.y), float(p.z)))

        self.traj_points = pts
        self.trja_frame_id = msg.header.frame_id or None
        self.traj_index = 0
        self.traj_sequence += 1
        self.get_logger().info(f"Received /uav/trajectory with {len(self.traj_points)} points (seq={self.traj_sequence}).")

    # === Helper: wait for FCU connection ===

    def wait_for_connection(self, timeout_sec: float = 30.0):
        """
        Wait until MAVROS is connected to the FCU.

        Condition:
        - self.current_state is not None
        - self.current_state.connected is True
        """
        self.get_logger().info("[INFO] wait for connection to FCU")
        start_time = self.get_clock().now()
        last_log_time = start_time

        while rclpy.ok():
            # One spin to process callbacks (state_cb)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check connection flag
            if self.current_state is not None and self.current_state.connected:
                self.get_logger().info("[INFO] Connected to FCU")
                return True

            # Status log every second
            now = self.get_clock().now()
            if (now - last_log_time) > Duration(seconds=1.0):
                last_log_time = now
                if self.current_state is None:
                    self.get_logger().info("[INFO] no state message received yet")
                else:
                    self.get_logger().warn(
                        f"FCU not connected (connected={self.current_state.connected}). Waiting..."
                    )

    # === Not used in this demo, but kept for later ===

    def arm(self):
        """
        Arming is manual in this demo.
        """
        raise NotImplementedError("arm() is not implemented for this demo.")


    # === Helper: publish one position setpoint ===

    def publish_position_setpoint(self, x: float, y: float, z: float, update_hold: bool = True):
        """Publish a single position setpoint to MAVROS."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.current_pose is not None:
            msg.header.frame_id = self.current_pose.header.frame_id
        else:
            msg.header.frame_id = "odom"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        qx, qy, qz, qw = self.quat_from_yaw(self.yaw_cmd if self.yaw_cmd is not None else 0.0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.setpoint_pub.publish(msg)

        if update_hold:
            self.hold_point = (x, y, z)

    # Math helper: 3D distance
    @staticmethod
    def dist3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    # Math helpers: yaw/quaternion conversions ===
    def yaw_from_quat(self, q):
        # q has fields x,y,z,w
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quat_from_yaw(self, yaw):
         # roll=pitch=0, yaw about Z
        return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))  # x,y,z,w



    # === Main periodic update loop ===

    def update(self):
        """
        Phases: wait_offboard, taking_off, holding, following_trajectory
        Monitoring connection and Flightmode while setting dynamic Hold Points
        Following trajectory by iterating trough Planners List of Trajectorys while checking goal distance
        """

        # No state yet; skip.
        if self.current_state is None:
            return

        now = self.get_clock().now()

        # Heartbeat check: lost MAVROS state messages?
        if self.last_state_time is None or (now - self.last_state_time) > Duration(seconds=2.0):
            if self.last_connected is not False:
                self.get_logger().warn("[WARN] lost MAVROS state heartbeat – treating as disconnected")
                self.last_connected = False
            return

        state = self.current_state

        # Log connection changes
        if self.last_connected is None or self.last_connected != state.connected:
            self.get_logger().info(
                f"[DEBUG] connected flag changed: {self.last_connected} -> {state.connected}"
            )
            self.last_connected = state.connected

        # Skip control if not connected
        if not state.connected:
            return

        # Log mode changes
        if self.last_mode is None or self.last_mode != state.mode:
            self.get_logger().info(
                f"[DEBUG] mode changed: {self.last_mode!r} -> {state.mode!r}"
            )
            self.last_mode = state.mode
        
        # Require valid pose
        if self.current_pose is None:
            return

        # Extract current pose
        cur_x = self.current_pose.pose.position.x
        cur_y = self.current_pose.pose.position.y
        cur_z = self.current_pose.pose.position.z
        cur = (cur_x, cur_y, cur_z)

        # Set reference pose once (start pose)
        if self.ref_pose is None:
            self.ref_pose = self.current_pose
            self.get_logger().info("[INFO] reference pose set from current local position")
            self.hold_point = cur
            # First phase: wait for OFFBOARD while streaming hold setpoints
            self.phase = "wait_offboard"

        # Extract reference pose = Home point
        ref_x = self.ref_pose.pose.position.x
        ref_y = self.ref_pose.pose.position.y
        ref_z = self.ref_pose.pose.position.z

        takeoff_height = float(self.get_parameter('takeoff_altitude').value)
        goal_tol = float(self.get_parameter('goal_tolerance').value)

        # Ensure phase initialized
        if self.phase is None:
            self.phase = "wait_offboard"

        # === Simple phase machine ===

        if self.phase == "wait_offboard":
            # Continuously publish hold setpoints so PX4 receives setpoints before OFFBOARD.
            hx, hy, hz = self.hold_point
            self.publish_position_setpoint(*self.hold_point, update_hold=False)

            # On OFFBOARD entry, begin ascent
            if state.mode == "OFFBOARD":
                self.phase = "taking_off"
                self.get_logger().info("[PHASE] OFFBOARD detected -> starting")

        elif self.phase == "taking_off":
            # If OFFBOARD is lost, return to waiting
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while going_up -> back to wait_offboard")
                self.phase = "wait_offboard"
                return

            base_X, base_Y, base_Z = self.hold_point
            target_hover = (base_X, base_Y, base_Z + takeoff_height)
            self.publish_position_setpoint(*target_hover, update_hold=False)

            # Check if target altitude reached (simple tolerance)
            reached = abs(cur[2] - target_hover[2]) < max(0.1, goal_tol)
            if reached:
                self.hold_point = target_hover
                self.phase = "holding"
                self.get_logger().info("[PHASE] at hover height -> switch to holding phase")
            else:
                self.phase = "taking_off"

        elif self.phase == "holding":
            # If OFFBOARD is lost, return to waiting
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while holding -> back to wait_offboard")
                self.phase = "wait_offboard"
                return

            hx, hy, hz = self.hold_point
            self.publish_position_setpoint(hx, hy, hz, update_hold=False)
            if self.traj_points:
                self.phase = "following_trajectory"
                self.get_logger().info("[PHASE] trajectory available -> switch to following_trajectory phase")

        elif self.phase == "following_trajectory":
            # If OFFBOARD is lost, return to waiting
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while following_trajectory -> back to wait_offboard")
                self.phase = "wait_offboard"
                return
            # get trajectory from planner node
            if not self.traj_points:
                self.get_logger().warn("[PHASE] no trajectory points available -> switch to holding phase")
                self.phase = "holding"
                return
            if self.traj_index >= len(self.traj_points):
                self.get_logger().info("[PHASE] reached end of trajectory -> switch to landing phase")
                self.phase = "holding"
                return
            # target set 
            target_point = self.traj_points[self.traj_index]
            self.publish_position_setpoint(*target_point, update_hold=True)
            # check if reached target point
            d = self.dist3(cur, target_point)
            if d < goal_tol:
                self.traj_index += 1
                if self.traj_index < len(self.traj_points):
                    self.get_logger().info(f"[PHASE] reached trajectory point {self.traj_index} -> moving to next point{self.traj_points[self.traj_index]}")
                else:
                    self.get_logger().info("[PHASE] reached final trajectory point -> switch to holding phase")

        elif self.phase == "done":
            # TODO impplement landing Method 
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while done -> back to wait_offboard")
                self.phase = "wait_offboard"
                return
            # Hold till landing command
            hx, hy, hz = self.hold_point
            self.publish_position_setpoint(hx, hy, hz, update_hold=False)
            self.get_logger().info("[PHASE] in done phase, holding position")



def main(args=None):
    rclpy.init(args=args)

    node = OffboardController()
    node.wait_for_connection()
    # Operator should arm and switch to OFFBOARD in QGC when ready.

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("OffboardController shutting down (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

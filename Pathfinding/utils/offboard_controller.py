#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class OffboardController(Node):
    """
    Basic offboard control demo.

    Procedure:
    - Arming and OFFBOARD mode are set manually in QGC.
    Node actions:
    - wait for FCU connection
    - wait for valid local pose
    - stream hold setpoints at start pose
    - upon OFFBOARD: ascend +1 m, return to start height, then hold
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

        # Mode gate helpers
        self.last_mode = None
        self.last_connected = None
        self.last_state_time = None  # time of last /mavros/state message

        # Simple sequence phases:
        # None -> wait_offboard -> taking_off -> landing -> done
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

    def wait_for_pose(self):
        """
        Placeholder for later use (not implemented).
        """
        raise NotImplementedError("wait_for_pose() is not implemented yet.")

    def arm(self):
        """
        Arming is manual in this demo.
        """
        raise NotImplementedError("arm() is not implemented for this demo.")

    def set_mode_offboard(self):
        """
        Mode switch to OFFBOARD is manual in this demo.
        """
        raise NotImplementedError("set_mode_offboard() is not implemented for this demo.")

    # === Helper: publish one position setpoint ===

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish a single position setpoint to MAVROS."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.current_pose is not None:
            msg.header.frame_id = self.current_pose.header.frame_id
        else:
            msg.header.frame_id = "map"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Neutral orientation (no yaw control)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.setpoint_pub.publish(msg)

    # === Main periodic update loop ===

    def update(self):
        """
        currently in demo state to be tested irl 
        planner node will be called instead of this simple up-down demo
        20 Hz update. Monitors connection and mode, sets reference pose once,
        streams hold setpoints until OFFBOARD, then ascend +1 m and return to reference height.
        """

        # No state yet; skip.
        if self.current_state is None:
            return

        now = self.get_clock().now()

        # Heartbeat check: lost MAVROS state messages?
        if self.last_state_time is None or (now - self.last_state_time) > Duration(seconds=1.0):
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

        # Set reference pose once (start pose)
        if self.ref_pose is None:
            self.ref_pose = self.current_pose
            self.get_logger().info("[INFO] reference pose set from current local position")
            # First phase: wait for OFFBOARD while streaming hold setpoints
            self.phase = "wait_offboard"

        # Extract current pose
        cur_z = self.current_pose.pose.position.z
        ref_x = self.ref_pose.pose.position.x
        ref_y = self.ref_pose.pose.position.y
        ref_z = self.ref_pose.pose.position.z

        # Ensure phase initialized
        if self.phase is None:
            self.phase = "wait_offboard"

        # === Simple phase machine ===

        if self.phase == "wait_offboard":
            # Continuously publish hold setpoints so PX4 receives setpoints before OFFBOARD.
            self.publish_position_setpoint(ref_x, ref_y, ref_z)

            # On OFFBOARD entry, begin ascent
            if state.mode == "OFFBOARD":
                self.phase = "taking_off"
                self.get_logger().info("[PHASE] OFFBOARD detected -> starting")

        elif self.phase == "following_trajectory":
            # If OFFBOARD is lost, return to waiting
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while following_trajectory -> back to wait_offboard")
                self.phase = "wait_offboard"
                return

        elif self.phase == "taking_off":
            # If OFFBOARD is lost, return to waiting
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while going_up -> back to wait_offboard")
                self.phase = "wait_offboard"
                return

            target_z = ref_z + 1.0
            self.publish_position_setpoint(ref_x, ref_y, target_z)

            # Check if target altitude reached (simple tolerance)
            if abs(cur_z - target_z) < 0.1:
                self.phase = "landing"
                self.get_logger().info("[PHASE] reached +1 m -> going_down")

        elif self.phase == "landing":
            # If OFFBOARD is lost, return to waiting
            if state.mode != "OFFBOARD":
                self.get_logger().warn("[PHASE] left OFFBOARD while going_down -> back to wait_offboard")
                self.phase = "wait_offboard"
                return

            self.publish_position_setpoint(ref_x, ref_y, ref_z)

            # Check if back at reference height
            if abs(cur_z - ref_z) < 0.1:
                self.phase = "done"
                self.get_logger().info("[PHASE] back at reference height -> done")

        elif self.phase == "done":
            # Maintain hold at reference height
            self.publish_position_setpoint(ref_x, ref_y, ref_z)


def main(args=None):
    rclpy.init(args=args)

    node = OffboardController()
    node.wait_for_connection()
    # Operator should arm and switch to OFFBOARD in QGC when ready.
    # Node handles the +1 m up/down sequence automatically.

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("OffboardController shutting down (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

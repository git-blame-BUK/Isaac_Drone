
#!/usr/bin/env python3
"""
OffboardController – minimal offboard skeleton for PX4 + MAVROS + ROS 2.

This script is NOT complete.
It shows structure and places where you should add your own logic.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import time  # may be used later in your own methods

# Message and service types from mavros_msgs + geometry_msgs
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class OffboardController(Node):
    """
    Basic offboard control skeleton.

    This node:
    - Stores current FCU state and local pose.
    - Provides publishers and service clients for offboard and arming
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
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
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
        # Arming service client
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        # Flight mode change service client
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        # Internal state
        self.current_state: State | None = None
        self.current_pose: PoseStamped | None = None
        self.ref_pose: PoseStamped | None = None

        # Timer for periodic logic (20 Hz) + state udpate 
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("OffboardController initialized. Waiting for FCU state and pose...")

    # === Callbacks ===

    def state_cb(self, msg: State):
        """Update cached FCU state."""
        self.current_state = msg

    def pose_cb(self, msg: PoseStamped):
        """Update cached local pose."""
        self.current_pose = msg

    # === Helper methods to implement step by step ===

    def wait_for_connection(self, timeout_sec: float = 30.0):
        """
        Wait until MAVROS is connected to the FCU.

        Condition:
        - self.current_state is not None
        - self.current_state.connected is True

        timeout_sec:
        - maximum wait time in seconds, ignored if you do not use it.
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

    def wait_for_pose(self):
        """
        Wait until a valid local pose is available.

        After that:
        - Copy current_pose into ref_pose.
        - Log 'Reference pose set'.
        """
        raise NotImplementedError("wait_for_pose() is not implemented yet.")

    def arm(self):
        """
        Arm the drone via MAVROS service.

        Steps (you should implement):
        - Wait for arming service to be available.
        - Send CommandBool request with value=True.
        - Wait for response and log success/failure.
        """
        raise NotImplementedError("arm() is not implemented yet.")

    def set_mode_offboard(self):
        """
        Switch PX4 mode to OFFBOARD via MAVROS service.

        Steps (you should implement):
        - Wait for set_mode service to be available.
        - Send SetMode request with custom_mode='OFFBOARD'.
        - Wait for response and log result.

        Note:
        - PX4 only accepts OFFBOARD if setpoints are already being published.
        """
        raise NotImplementedError("set_mode_offboard() is not implemented yet.")

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """
        Publish a single position setpoint to MAVROS.

        Steps (you should implement):
        - Create PoseStamped.
        - Set header.stamp to current time and header.frame_id
          to match your local_position frame (e.g. 'map' or 'local_origin').
        - Set pose.position.x/y/z to the given values.
        - Set orientation to identity (0, 0, 0, 1) for now.
        - Publish via self.setpoint_pub.
        """
        raise NotImplementedError("publish_position_setpoint() is not implemented yet.")

    # === Main periodic update loop ===

    def update(self):
        """
        Called at 20 Hz by the timer.

        Here you will later:
        - Publish position setpoints regularly.
        - Implement a simple state machine (e.g. idle, takeoff, hover, land).

        For now:
        - Only log basic information.
        """

        # If we do not have any state yet, do nothing
        if self.current_state is None:
            return

        # Mode gate to Offboard
        if self.current_state.mode != "OFFBOARD":
            return

        # Later you will:
        # - Call publish_position_setpoint(...) here.
        # - Use an internal phase/state variable to decide target positions.

def main(args=None):
    rclpy.init(args=args)

    node = OffboardController()
    node.wait_for_connection()
    # Later you can call here in sequence:
    # - node.wait_for_connection()
    # - node.wait_for_pose()
    # - node.arm()
    # - node.set_mode_offboard()
    # after you have implemented these methods.

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("OffboardController shutting down (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# interface for managing missions in a pathfinding application

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time
from geometry_msgs.msg import PoseStamped


class MissionManager(Node):
    """
    Publishes a single PoseStamped goal to /uav/goal_pose.
    Uses TRANSIENT_LOCAL durability (ROS2 'latched' behavior) so the goal isn't missed
    if the planner subscribes slightly later.
    """

    def __init__(self):
        super().__init__("mission_manager")

        qos_latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.goal_pub = self.create_publisher(PoseStamped, "/uav/goal_pose", qos_latched)

        # parameters (set via CLI or defaults)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("x", input("enter goal x e.g 1.0 for 1m forward  : ") or 0.0)
        self.declare_parameter("y", input("enter goal y e.g 1.0 for 1m right  : ") or 0.0)
        self.declare_parameter("z", input("enter goal z e.g 1.0 for 1m up     : ") or 0.5)

        # publish a few times so it’s super robust
        self._count = 0
        self._timer = self.create_timer(0.2, self._publish_goal_timer)

        self.get_logger().info("MissionManager started. Publishing goal to /uav/goal_pose...")

    def _publish_goal_timer(self):
        frame_id = str(self.get_parameter("frame_id").value)
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        z = float(self.get_parameter("z").value)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0  # no yaw command here

        self.goal_pub.publish(msg)
        
        # publish 10 times (~2 seconds) then exit
        try:
            if self._count < 10:
                self._count += 1
            self.goal_pub.publish(msg)
            
            i = input ("Goal published want to publish again ? y/n : ")
            self.get_logger().info(f"Published goal #{self._count}: ({x}, {y}, {z}) in frame '{frame_id}'")
            if i == 'y':
                self._count = 0
            elif i == 'n' or self._count >= 10:
                self.get_logger().info("Goal publishing complete. Shutting down MissionManager.")
                rclpy.shutdown()
        except: 
            # Ctl+C during sleep will raise exception 
            self.get_logger().info("something went wrong while publishing goal")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()

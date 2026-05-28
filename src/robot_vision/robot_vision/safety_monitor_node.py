#!/usr/bin/env python3
"""
Safety Monitor Node
Subscribes to /person_detected.
If True — sends zero velocity to stop the robot.
If False — does nothing, allows normal teleop commands through.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        # ── State ─────────────────────────────────────────
        self.person_detected = False

        # ── Subscriber — person detection result ──────────
        self.detection_sub = self.create_subscription(
            Bool,
            '/person_detected',
            self.detection_callback,
            10)

        # ── Publisher — velocity commands to robot ────────
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10)

        # ── Timer — sends stop command at 20Hz when person detected
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Safety monitor node ready.')

    def detection_callback(self, msg):
        was_detected = self.person_detected
        self.person_detected = msg.data

        if self.person_detected and not was_detected:
            self.get_logger().warn('PERSON DETECTED — sending stop command!')
        elif not self.person_detected and was_detected:
            self.get_logger().info('Person gone — robot may resume.')

    def timer_callback(self):
        if self.person_detected:
            # Send zero velocity to stop the robot
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_footprint'
            stop_msg.twist.linear.x  = 0.0
            stop_msg.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
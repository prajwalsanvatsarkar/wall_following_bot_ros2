#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.front = float('inf')
        self.right1 = float('inf')
        self.right2 = float('inf')
        self.state = 'APPROACH'
        self.turn_start_time = None
        self.turn_duration = 3.2  # seconds for ~90° at 0.5 rad/s

        self.msg = Twist()

    def scan_callback(self, scan: LaserScan):
        if len(scan.ranges) >= 275:
            self.front = scan.ranges[0]
            self.right1 = scan.ranges[266]
            self.right2 = scan.ranges[274]

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == 'APPROACH':
            if self.front > 0.30:
                self.msg.linear.x = 0.07
                self.msg.angular.z = 0.0
                self.get_logger().info("Approaching wall...")
            else:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.turn_start_time = now
                self.state = 'TURN_LEFT_INIT'
                self.get_logger().info("Wall ahead — turning left")

        elif self.state == 'TURN_LEFT_INIT':
            elapsed = now - self.turn_start_time
            if elapsed < self.turn_duration:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.5
                self.get_logger().info("Turning left...")
            else:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.state = 'FOLLOW_WALL'
                self.get_logger().info("Turn complete — following wall")

        elif self.state == 'FOLLOW_WALL':
            if self.front < 0.30:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.5
                self.get_logger().info("Obstacle ahead — turning left")

            elif (self.right1 - self.right2) >= 0.008:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.10
                self.get_logger().info("Too far from wall — turning left")

            elif (self.right1 - self.right2) <= 0.008:
                self.msg.linear.x = 0.05
                self.msg.angular.z = 0.0
                self.get_logger().info("Following wall")

                if self.right2 >= 0.38:
                    self.msg.linear.x = 0.05
                    self.msg.angular.z = -0.30
                    self.get_logger().info("Corner detected — turning right")

            elif (self.right2 - self.right1) <= 0.004:
                self.msg.linear.x = 0.05
                self.msg.angular.z = 0.0
                self.get_logger().info("Aligned")

        self.cmd_pub.publish(self.msg)


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


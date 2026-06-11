#!/usr/bin/env python3

# MrPeterBot — Circle Test Node
# Continuously updates theta_ref to make the robot go in circles
# theta_ref = theta0 + angular_offset → robot always tries to turn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math


class CircleTest(Node):

    def __init__(self):
        super().__init__('circle_test')

        # ── Parameters ────────────────────────────────────────────────────────
        self.velocity    = 0.2    # forward speed [m/s]
        self.turn_offset = 0.3    # heading offset [rad] — bigger = tighter circle

        # Current heading
        self.theta0 = 0.0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.v_ref_pub = self.create_publisher(
            Float64, '/v_ref', 10)

        self.theta_ref_pub = self.create_publisher(
            Float64, '/theta_ref', 10)

        # ── Timer — 50Hz ──────────────────────────────────────────────────────
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Circle Test Node started!')
        self.get_logger().info(f'velocity={self.velocity} turn_offset={self.turn_offset} rad')

    def odom_callback(self, msg: Odometry):
        # Extract current heading from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta0 = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # Always set theta_ref ahead of current heading → continuous turn
        theta_ref = self.theta0 + self.turn_offset

        # Wrap to [-π, π]
        theta_ref = math.atan2(math.sin(theta_ref), math.cos(theta_ref))

        # Publish velocity
        v_msg = Float64()
        v_msg.data = self.velocity
        self.v_ref_pub.publish(v_msg)

        # Publish heading reference
        t_msg = Float64()
        t_msg.data = theta_ref
        self.theta_ref_pub.publish(t_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on exit
        node.v_ref_pub.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
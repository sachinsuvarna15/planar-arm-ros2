#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster


class FKSolver(Node):
    def __init__(self):
        super().__init__('fk_solver')

        self.l1 = 0.4
        self.l2 = 0.3
        self.l3 = 0.2

        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0

        self.joint_received = False

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            Point,
            '/end_effector_position',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.compute_fk)   # 10 Hz

        self.get_logger().info('FK Solver started. Waiting for /joint_states...')

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < 3:
            self.get_logger().warn('Received /joint_states with fewer than 3 joints.')
            return

        self.theta1 = msg.position[0]
        self.theta2 = msg.position[1]
        self.theta3 = msg.position[2]
        self.joint_received = True

    def compute_fk(self):
        if not self.joint_received:
            return

        x = (
            self.l1 * math.cos(self.theta1) +
            self.l2 * math.cos(self.theta1 + self.theta2) +
            self.l3 * math.cos(self.theta1 + self.theta2 + self.theta3)
        )

        y = (
            self.l1 * math.sin(self.theta1) +
            self.l2 * math.sin(self.theta1 + self.theta2) +
            self.l3 * math.sin(self.theta1 + self.theta2 + self.theta3)
        )

        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0.0
        self.publisher.publish(point_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base_link'
        tf_msg.child_frame_id = 'computed_end_effector'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f'Angles: {self.theta1:.2f}, {self.theta2:.2f}, {self.theta3:.2f} '
            f'-> x={x:.2f}, y={y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FKSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

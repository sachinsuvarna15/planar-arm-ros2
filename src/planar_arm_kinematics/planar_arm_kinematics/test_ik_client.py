#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from planar_arm_kinematics.srv import ComputeIK

class IKClient(Node):

    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(ComputeIK, 'compute_ik')

        # wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')

    def send_request(self, x, y):
        req = ComputeIK.Request()
        req.target_x = x
        req.target_y = y

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main():
    rclpy.init()
    node = IKClient()

    tests = [(0.5, 0.4), (0.0, 0.0), (1.5, 1.5)]

    for t in tests:
        response = node.send_request(t[0], t[1])

        print(f"\nInput: {t}")
        print(f"Success: {response.success}")
        print(f"Message: {response.message}")
        print(f"Theta1: {response.theta1}")
        print(f"Theta2: {response.theta2}")
        print(f"Theta3: {response.theta3}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()

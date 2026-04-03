#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from planar_arm_kinematics.srv import ComputeIK
import math

class IKSolver(Node):

    def __init__(self):
        super().__init__('ik_solver')

        self.srv = self.create_service(
            ComputeIK,
            'compute_ik',
            self.callback
        )

        self.L1 = 0.4
        self.L2 = 0.3

        self.get_logger().info("IK Service Ready ")

    def callback(self, request, response):

        x = request.target_x
        y = request.target_y

        d = math.sqrt(x**2 + y**2)

        if d > (self.L1 + self.L2):
            response.success = False
            response.message = "Target unreachable"
            return response

        cos_t2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2*self.L1*self.L2)
        
        cos_t2 = max(min(cos_t2, 1.0), -1.0)
        theta2 = math.acos(cos_t2)

        theta1 = math.atan2(y, x) - math.atan2(
            self.L2*math.sin(theta2),
            self.L1 + self.L2*math.cos(theta2)
        )

        response.success = True
        response.theta1 = theta1
        response.theta2 = theta2
        response.theta3 = 0.0
        response.message = "Success"

        self.get_logger().info(
            f"Target ({x:.2f},{y:.2f}) -> theta1={theta1:.2f}, theta2={theta2:.2f}"
        )

        return response


def main():
    rclpy.init()
    node = IKSolver()
    rclpy.spin(node)  
    rclpy.shutdown()


if __name__ == '__main__':
    main()

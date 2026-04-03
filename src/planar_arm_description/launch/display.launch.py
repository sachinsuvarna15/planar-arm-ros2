#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('planar_arm_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'planar_arm.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'planar_arm.rviz')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])

#!/usr/bin/env python3
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('test_api'),
        'config',
        'servo_test.yaml'
        )

    test_servos_node = Node(
            package="test_api",
            namespace="",
            executable="test_servos",
            name="test_servos",
            parameters=[config],
            output="screen",
        )

    return LaunchDescription([
        test_servos_node,
    ])
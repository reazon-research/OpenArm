#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openarm_bilateral',
            executable='bilateral_openarm_main',
            output='screen',
            parameters=[]
        )
    ])

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    urdf_path = '/home/astrobotics/astro3/src/astro_launch/urdf/rover.urdf'

    return LaunchDescription([
        # Only publish TFs from URDF, no GUI stuff here
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_path])
            }]
        )
    ])

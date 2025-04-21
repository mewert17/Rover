#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  


def generate_launch_description():
    pkg_share = FindPackageShare('astro_launch')


    # paths
    nav2_params = '/home/matt/Desktop/luna/src/astro_launch/params/nav2_params.yaml'
    arena_map   = '/home/matt/Desktop/luna/src/astro_launch/params/arena_map.yaml'
    urdf_file   = '/home/matt/Desktop/luna/src/astro_launch/urdf/rover.urdf'

    # URDF → robot_description param
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', urdf_file])
        }]
    )

        # publish a zero‑offset odom frame under map
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=[
            '0', '0', '0',    # x y z
            '0', '0', '0',    # roll pitch yaw (radians)
            'map',            # parent frame
            'odom'            # child frame
        ]
    )

    motor_node = Node(
        package='astro_launch',
        executable='motor_command_node',
        name='motor_command_node',
        output='screen'
    )

    #beacon_node = Node(
    #    package='astro_launch',
     #   executable='beacon_localization',
      #  name='beacon_localization',
       # output='screen'
    #)

    # Nav2 nodes
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', output='screen',
        parameters=[nav2_params]
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', output='screen',
        parameters=[nav2_params]
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=[nav2_params]
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
        parameters=[nav2_params]
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server', output='screen',
        parameters=[{'use_sim_time': False, 'yaml_filename': arena_map}]
    )

    # new behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )



    return LaunchDescription([
        # URDF & TF publisher
        robot_state_publisher,
        static_map_to_odom,     # hmmm

        motor_node,
        #beacon_node,

        # Nav2 core
        bt_navigator,
        planner_server,
        controller_server,
        lifecycle_manager,
        map_server,
        behavior_server,

    ])

#!/usr/bin/env python

"""Launch Webots Universal Robot simulation."""

import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


PACKAGE_NAME = 'sofar_ur_webots'


def generate_launch_description():
    world = LaunchConfiguration('world')

    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'webots_ur5e_description.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )

    ur1_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace='ur1',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5_1'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            ros2_control_params
        ]
    )

    ur2_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace='ur2',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'UR5_2'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            ros2_control_params
        ]
    )

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    ur1_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur1/controller_manager'] + controller_manager_timeout,
    )

    ur1_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur1/controller_manager'] + controller_manager_timeout,
    )

    ur2_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller', '-c', 'ur2/controller_manager'] + controller_manager_timeout,
    )

    ur2_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster', '-c', 'ur2/controller_manager'] + controller_manager_timeout,
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='universal_robots.wbt',
            description=f'Choose one of the world files from `/{PACKAGE_NAME}/world` directory'
        ),
        webots,
        ur1_driver,
        ur2_driver,
        ur1_joint_state_broadcaster_spawner,
        ur2_joint_state_broadcaster_spawner,
        ur1_trajectory_controller_spawner,
        ur2_trajectory_controller_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])

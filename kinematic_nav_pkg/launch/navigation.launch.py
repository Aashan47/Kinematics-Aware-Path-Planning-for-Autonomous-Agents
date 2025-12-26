"""
Launch file for Kinematics-Aware Navigation

This launch file demonstrates:
1. Python-based ROS2 launch system
2. Parameter loading from YAML
3. Node configuration and namespacing
4. Conditional launching based on arguments

Usage:
    ros2 launch kinematic_nav_pkg navigation.launch.py
    ros2 launch kinematic_nav_pkg navigation.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""

    # Get package directory for config files
    pkg_dir = get_package_share_directory('kinematic_nav_pkg')
    params_file = os.path.join(pkg_dir, 'config', 'navigator_params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Navigator node
    navigator_node = Node(
        package='kinematic_nav_pkg',
        executable='point_to_point_navigator',
        name='point_to_point_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        remappings=[
            # Remap topics if needed for specific robots
            # ('/odom', '/robot/odom'),
            # ('/cmd_vel', '/robot/cmd_vel'),
        ]
    )

    # Log startup info
    startup_log = LogInfo(
        msg=['Starting Kinematics-Aware Navigation with params from: ', params_file]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        log_level_arg,

        # Logging
        startup_log,

        # Nodes
        navigator_node,
    ])

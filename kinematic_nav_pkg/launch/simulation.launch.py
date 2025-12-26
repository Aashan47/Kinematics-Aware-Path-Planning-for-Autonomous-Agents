"""
Gazebo Simulation Launch File

Launches a complete simulation environment with:
1. Gazebo Ignition (or Classic) with TurtleBot3
2. RViz2 for visualization
3. Our navigation node

Prerequisites:
    sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description
    export TURTLEBOT3_MODEL=burger

Usage:
    ros2 launch kinematic_nav_pkg simulation.launch.py
    ros2 launch kinematic_nav_pkg simulation.launch.py world:=empty_world.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the simulation launch description."""

    # Package directories
    pkg_dir = get_package_share_directory('kinematic_nav_pkg')
    params_file = os.path.join(pkg_dir, 'config', 'navigator_params.yaml')

    # TurtleBot3 packages (must be installed)
    tb3_gazebo_pkg = 'turtlebot3_gazebo'
    tb3_desc_pkg = 'turtlebot3_description'

    # Environment variable for TurtleBot3 model
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    # Launch Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',
        description='Gazebo world file'
    )

    # Include TurtleBot3 Gazebo launch
    # This spawns the robot model with sensors (LiDAR, odometry)
    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(tb3_gazebo_pkg),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    # RViz2 Node
    rviz_config_file = os.path.join(pkg_dir, 'config', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file)
                  else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Our Navigator Node
    navigator_node = Node(
        package='kinematic_nav_pkg',
        executable='point_to_point_navigator',
        name='point_to_point_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': True}
        ],
    )

    # Goal Publisher (for testing) - publishes a goal after 5 seconds
    goal_publisher = Node(
        package='ros2',
        executable='topic',
        name='goal_publisher',
        output='screen',
        arguments=[
            'pub', '--once', '/goal_pose',
            'geometry_msgs/msg/PoseStamped',
            '{header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'
        ],
        prefix='bash -c "sleep 5 && $0 $@"'
    )

    return LaunchDescription([
        # Environment
        set_tb3_model,

        # Arguments
        use_rviz_arg,
        world_arg,

        # Gazebo + Robot
        tb3_gazebo_launch,

        # Visualization
        rviz_node,

        # Navigation
        navigator_node,
    ])

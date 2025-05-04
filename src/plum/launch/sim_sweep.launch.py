import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Configurable map argument
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='./apartment.yaml',
        description='Path to the map YAML file'
    )

    map_config = LaunchConfiguration('map')

    # Paths to other launch files (assuming they’re in plum/launch/)
    plum_launch_dir = os.path.join(get_package_share_directory('plum'), 'launch')

    sim_launch_path = os.path.join(plum_launch_dir, 'sim_launch.launch.py')
    localization_launch_path = os.path.join(plum_launch_dir, 'localization_launch.py')
    joystick_launch_path = os.path.join(plum_launch_dir, 'joystick.launch.py')

    # Launch sequence with delays
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_config
        }.items()
    )

    planner_node = Node(
        package='plum_nodes',
        executable='coverage_planner_node',
        name='coverage_planner',
        output='screen'
    )

    follower_node = Node(
        package='plum_nodes',
        executable='rotate_drive_follower',
        name='rotate_drive_follower',
        output='screen'
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_path)
    )

    return LaunchDescription([
        map_arg,

        sim_launch,
        TimerAction(period=8.0, actions=[]),

        localization_launch,
        TimerAction(period=8.0, actions=[]),

        planner_node,
        TimerAction(period=8.0, actions=[]),

        follower_node,
        TimerAction(period=8.0, actions=[]),

        joystick_launch,
    ])

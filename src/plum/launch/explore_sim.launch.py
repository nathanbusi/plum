from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    plum_dir = get_package_share_directory('plum')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Launch sim
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plum_dir, 'launch', 'sim_launch.launch.py')
        )
    )

    # Launch SLAM Toolbox after 6s (from your own plum package)
    slam_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(plum_dir, 'launch', 'online_async.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # Launch Nav2 after another 6s (12s total)
    nav2_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'autostart': 'true',
                    'params_file': os.path.join(plum_dir, 'config', 'nav2_params.yaml'),
                    'map_subscribe_transient_local': 'true'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        slam_launch,
        nav2_launch
    ])

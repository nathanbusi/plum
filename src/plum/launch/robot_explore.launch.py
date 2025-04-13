from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    plum_dir = get_package_share_directory('plum')

    # Launch SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plum_dir, 'launch', 'online_async.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Launch Nav2 after 6s
    nav2_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(plum_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'autostart': 'true',
                    'params_file': os.path.join(plum_dir, 'config', 'nav2_params.yaml'),
                    'map_subscribe_transient_local': 'true'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        slam_launch,
        nav2_launch
    ])

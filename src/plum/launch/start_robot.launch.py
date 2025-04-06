import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_path = get_package_share_directory('plum')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'robot_launch.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'lidar.launch.py')
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'camera.launch.py')
        )
    )

    delayed_lidar = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_launch,
            on_start=[lidar_launch],
        )
    )

    delayed_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=lidar_launch,
            on_start=[camera_launch],
        )
    )

    return LaunchDescription([
        robot_launch,
        delayed_lidar,
        delayed_camera,
    ])

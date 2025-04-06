import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    # Delay lidar and camera to ensure robot state is up first
    delayed_lidar = TimerAction(
        period=3.0,
        actions=[lidar_launch]
    )

    delayed_camera = TimerAction(
        period=3.0,
        actions=[camera_launch]
    )

    return LaunchDescription([
        robot_launch,
        delayed_lidar,
        delayed_camera
    ])

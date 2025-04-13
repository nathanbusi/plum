import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'planner_frequency': 1.0,
                'progress_timeout': 30.0,
                'potential_scale': 1.0,
                'gain_scale': 1.0,
                'frontier_travel_point': 1,
                'costmap_topic': '/global_costmap/costmap'
            }]
        )
    ])

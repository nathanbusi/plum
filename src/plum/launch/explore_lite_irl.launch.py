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
                'use_sim_time': False,
                'planner_frequency': 0.1,
                'progress_timeout': 90.0,
                'potential_scale': 3.0,
                'gain_scale': 1.0,
                'frontier_travel_point': 1,
                'min_frontier_size': 0.01,  # in meters
                'costmap_topic': '/global_costmap/costmap'
            }]
        )
    ])

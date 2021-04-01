"""TODO"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    log_level = LaunchConfiguration('log_level', default='info')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description="If true, use simulated clock"),
        DeclareLaunchArgument(
            'log_level',
            default_value=log_level,
            description="Log level of all nodes launched by this script"),

        # TODO: Figure out what RGB-D camera to use to real-life test
        # TODO: Add camera publisher
        # TODO: Convert RGB+Depth images into XYZRGB point cloud
        # TODO: Add camera tf (w.r.t robot_base/world)

    ])

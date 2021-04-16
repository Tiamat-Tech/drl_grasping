"""Launch an RGB-D camera. Currently configured for RealSense device."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    log_level = LaunchConfiguration('log_level', default='info')
    config_rs_camera = LaunchConfiguration('config_rs_camera', default=os.path.join(get_package_share_directory('drl_grasping'),
                                                                                    'config', 'rs_camera.yaml'))

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
        DeclareLaunchArgument(
            'config_rs_camera',
            default_value=config_rs_camera,
            description="Config for RealSense camera"),

        # RealSense node
        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_namespace="/rs_camera",
            output='screen',
            parameters=[config_rs_camera],
            remappings=[('rs_camera/camera/pointcloud', 'rgbd_camera/points'),
                        ('rs_camera/camera/image_raw', 'rgbd_camera/image'),
                        ('rs_camera/camera/aligned_depth_to_color/image_raw', 'rgbd_camera/depth_image')],
        ),

        # Transformation (world <-> rs_camera)
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_rs_camera',
            node_namespace='',
            output='screen',
            parameters=[],
            # TODO: camera pose
            arguments=['0.0', '0.0', '0.0',
                       '0.0', '0.0', '0.0',
                       'world', 'rs_camera'],
            remappings=[],
        ),
    ])

"""Launch setup for real world evaluation of agents trained inside simulation."""

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
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory('drl_grasping'),
                                                                            'launch', 'rviz.rviz'))
    log_level = LaunchConfiguration('log_level', default='info')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description="If true, use simulated clock"),
        DeclareLaunchArgument(
            'config_rviz2',
            default_value=config_rviz2,
            description="Path to config for RViz2. If empty, RViz2 will be disabled"),
        DeclareLaunchArgument(
            'log_level',
            default_value=log_level,
            description="Log level of all nodes launched by this script"),

        # Launch camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('drl_grasping'),
                              'launch', 'real', 'camera.launch.py')]),
            launch_arguments=[('use_sim_time', use_sim_time),
                              ('log_level', log_level)]),

        # Transformation (world <-> panda)
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher_panda',
            node_namespace='',
            output='screen',
            parameters=[],
            arguments=['0.0', '0.0', '0.0',
                       '0.0', '0.0', '0.0',
                       'world', 'panda_link0'],
            remappings=[],
        ),

        # # RViz2
        # Node(
        #     package='rviz2',
        #     node_executable='rviz2',
        #     node_name='rviz2',
        #     node_namespace='',
        #     output='screen',
        #     arguments=['--display-config', config_rviz2],
        #     parameters=[robot_description,
        #                 robot_description_semantic,
        #                 kinematics,
        #                 {'use_sim_time': use_sim_time}])

        # MoveIt2 move_group action server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('panda_moveit2_config'),
                              'launch', 'move_group_action_server.launch.py')]),
            launch_arguments=[('use_sim_time', use_sim_time),
                              ('config_rviz2', config_rviz2),
                              ('log_level', log_level)]),
    ])

## For testing purposes (when robot is not connected)
# ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "header:
#   stamp:
#     sec: $(date '+%s')
#     nanosec: $(date '+%N')
#   frame_id: 'world'
# name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, panda_finger_joint1, panda_finger_joint2]
# position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

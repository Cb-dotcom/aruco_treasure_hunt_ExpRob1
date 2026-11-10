"""Gazebo simulation of the erl1 robot in the assignment world.

Same as erl1/launch/spawn_assignment.launch.py, but the bridge is configured from
config/gz_bridge.yaml (the erl1 file has a typo in its /clock entry).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    erl1 = get_package_share_directory('erl1')
    bringup = get_package_share_directory('treasure_hunt_bringup')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(erl1, 'urdf', 'my_robot.urdf')]), value_type=str)

    return LaunchDescription([
        # aruco_box_* models live in ~/gazebo_models; erl1 meshes are found via package://erl1
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.expanduser('~/gazebo_models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(erl1)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(erl1, 'launch', 'world.launch.py')),
            launch_arguments={'world': 'assignment.sdf'}.items()),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'my_robot', '-topic', 'robot_description', '-z', '0.5']),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(bringup, 'config', 'gz_bridge.yaml'),
                'use_sim_time': True,
            }]),

        Node(
            package='ros_gz_image', executable='image_bridge', arguments=['/camera/image'],
            parameters=[{'use_sim_time': True}]),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[os.path.join(erl1, 'config', 'ekf.yaml'), {'use_sim_time': True}]),

        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(erl1, 'rviz', 'urdf.rviz')],
            parameters=[{'use_sim_time': True}]),
    ])

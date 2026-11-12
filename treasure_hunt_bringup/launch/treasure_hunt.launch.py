"""Treasure-hunt stack: detector, motion controller and hunt manager in one container.

    ros2 launch treasure_hunt_bringup treasure_hunt.launch.py                # Gazebo
    ros2 launch treasure_hunt_bringup treasure_hunt.launch.py robot:=rosbot  # real ROSbot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('treasure_hunt_bringup'), 'config',
        [LaunchConfiguration('robot'), '.yaml']])

    def component(plugin, name):
        return ComposableNode(
            package='treasure_hunt', plugin='treasure_hunt::' + plugin, name=name,
            parameters=[params])

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', choices=['sim', 'rosbot']),
        ComposableNodeContainer(
            name='treasure_hunt', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen',
            composable_node_descriptions=[
                component('ArucoDetectorComponent', 'aruco_detector'),
                component('MotionControllerComponent', 'motion_controller'),
                component('HuntManagerComponent', 'hunt_manager'),
            ]),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_ros2',
            executable='joystick_ros2',
        ),
        Node(
            package='lawn_mower_control',
            executable='manual_control'
        ),
        #Node(
        #    package='lawn_mower',
        #    executable='mowing_planner'
        #)
    ])

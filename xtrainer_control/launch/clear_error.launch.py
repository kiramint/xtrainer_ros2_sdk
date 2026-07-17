"""
一键清除双臂报警

Usage: ros2 launch xtrainer_control clear_error.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="xtrainer_control",
            executable="clear_error",
            name="clear_error",
            output="screen",
            parameters=[{
                "namespaces": ["Arm1", "Arm2"],
            }],
        ),
    ])
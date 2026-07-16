"""
一键去使能双臂 (Arm1, Arm2)

Usage: ros2 launch xtrainer_control disable.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="xtrainer_control",
            executable="disable_arms",
            name="disable_arms",
            output="screen",
            parameters=[{
                "namespaces": ["Arm1", "Arm2"],
            }],
        ),
    ])
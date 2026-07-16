"""
一键使能双臂 (Arm1, Arm2)

Usage: ros2 launch xtrainer_control enable.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="xtrainer_control",
            executable="enable_arms",
            name="enable_arms",
            output="screen",
            parameters=[{
                "namespaces": ["Arm1", "Arm2"],
            }],
        ),
    ])
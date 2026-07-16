"""
一键使能 + 拖拽示教双臂 (Arm1, Arm2)

Usage: ros2 launch xtrainer_control enable_and_drag.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="xtrainer_control",
            executable="enable_and_drag",
            name="enable_and_drag",
            output="screen",
            parameters=[{
                "namespaces": ["Arm1", "Arm2"],
            }],
        ),
    ])
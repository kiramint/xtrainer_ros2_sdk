"""
一键张开双臂夹爪

用法:
    ros2 launch xtrainer_gripper gripper_open.launch.py
    ros2 launch xtrainer_gripper gripper_open.launch.py speed:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed_arg = DeclareLaunchArgument(
        "speed", default_value="1.0",
        description="开爪速度 (0.0~1.0)"
    )

    return LaunchDescription([
        speed_arg,
        Node(
            package="xtrainer_gripper",
            executable="gripper_open_arms",
            name="gripper_open_arms",
            output="screen",
            parameters=[{
                "speed": LaunchConfiguration("speed"),
            }],
        ),
    ])
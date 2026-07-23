import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("x_trainer", package_name="moveit_test")
        .moveit_cpp(
            file_path=get_package_share_directory("xtrainer_task")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    params = moveit_config.to_dict()

    goto_pose_node = Node(
        name="goto_pose_node",
        package="xtrainer_task",
        executable="goto_pose",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([
        goto_pose_node,
    ])

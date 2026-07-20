"""
Launch xtrainer_task with all MoveIt configs loaded as node parameters.

This mirrors the pattern from the official moveit_py tutorial:
  - Load configs via MoveItConfigsBuilder
  - Pass them as Node parameters (so MoveItPy reads from parameter server)

Usage
-----
    ros2 launch xtrainer_task start.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    moveit_py_node = Node(
        name="xtrainer_task_moveit",
        package="xtrainer_task",
        executable="start",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([moveit_py_node])
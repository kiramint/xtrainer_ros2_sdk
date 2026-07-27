"""
Launch xtrainer_task with MoveItPy + RViz visualization.

MoveItPy runs as an in-process planning backend and publishes:
  - /moveit_cpp/monitored_planning_scene  (robot state + planning scene)
  - /display_planned_path                 (planned trajectory, published by RobotMover)

RViz uses PlanningSceneDisplay (NOT MotionPlanningDisplay which requires move_group).

Can coexist with move_group (demo.launch.py) as long as only ONE sends trajectory
execution goals to /ArmX_controller/follow_joint_trajectory at a time.

Prerequisites
-------------
    ros2 launch xtrainer_control start_open_bottle.launch.py   (driver + bridge + gripper)

Usage
-----
    ros2 launch xtrainer_task start_open_bottle.launch.py
    ros2 launch xtrainer_task start_open_bottle.launch.py use_rviz:=false
"""

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

    moveit_py_node = Node(
        name="xtrainer_task_moveit",
        package="xtrainer_task",
        executable="start_open_bottle",
        output="screen",
        parameters=[params],
    )

    rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization",
    )

    rviz_config = os.path.join(
        get_package_share_directory("xtrainer_task"),
        "config",
        "xtrainer.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="xtrainer_rviz",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        rviz_arg,
        moveit_py_node,
        rviz_node,
    ])

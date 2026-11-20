"""一键拉起 XTrainer 驱动和 MoveIt，供 agent 直接看话题。

包含：
  1. xtrainer_control/start.launch.py
     双臂驱动、/joint_states、FollowJointTrajectory、三路彩色和对齐深度、
     点云、夹爪、手眼 TF。抱闸由 cr_robot_ros2_node 自己解锁和锁定。
     这里已经有一个 robot_state_publisher。
  2. xtrainer_moveit/move_group.launch.py
  3. xtrainer_moveit/moveit_rviz.launch.py，use_rviz 默认 true

不包含 demo.launch.py。那个文件会再起一个 robot_state_publisher，
还会发静态虚拟关节，和上面的 TF 打架。

MCP 进程不在这个 launch 里。stdio 必须独占 stdout，由 agent 另开进程启动。
不要同时跑 xtrainer_task 里会执行轨迹的脚本，执行器只能有 move_group 这一份。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    control_share = get_package_share_directory("xtrainer_control")
    moveit_share = get_package_share_directory("xtrainer_moveit")

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_rviz",
            default_value=True,
            description="为 true 时同时打开 MoveIt RViz",
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_share, "launch", "start.launch.py")
            )
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_share, "launch", "move_group.launch.py")
            )
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_share, "launch", "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return ld

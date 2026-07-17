"""
demo.launch.py — 用于 xtrainer_bridge 架构的 MoveIt demo 启动

与 moveit_configs_utils 默认 generate_demo_launch 的区别:
  - 不启动 ros2_control_node / 不 spawn_controllers,
    因为机械臂控制由 xtrainer_bridge 直接桥接到 Dobot ServoJ。
  - 若启动 ros2_control_node (JointTrajectoryController), 会在
    /ArmX_controller/follow_joint_trajectory 上与 bridge 的 action server 冲突,
    导致 MoveIt 报 "unknown goal response / unknown result response"。

前置条件: xtrainer_bridge 必须已经在运行
  (例如: ros2 launch xtrainer_control start.launch.py)。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("x_trainer", package_name="moveit_test").to_moveit_configs()
    package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # 1) 静态虚拟关节 TF
    virtual_joints_launch = package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 2) robot_state_publisher
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_path / "launch/rsp.launch.py")),
        )
    )

    # 3) move_group
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_path / "launch/move_group.launch.py")),
        )
    )

    # 4) RViz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 5) warehouse (可选)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_path / "launch/warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    return ld

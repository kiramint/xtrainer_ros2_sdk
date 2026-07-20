import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 从环境变量或默认值获取参数
    ns = os.getenv("GRIPPER_NS", "gripper")

    left_port = os.getenv("GRIPPER_LEFT_PORT", "/dev/ttyUSB0")
    left_id = os.getenv("GRIPPER_LEFT_ID", "21")
    left_min = os.getenv("GRIPPER_LEFT_MIN", "1981")
    left_max = os.getenv("GRIPPER_LEFT_MAX", "3069")

    right_port = os.getenv("GRIPPER_RIGHT_PORT", "/dev/ttyUSB1")
    right_id = os.getenv("GRIPPER_RIGHT_ID", "22")
    right_min = os.getenv("GRIPPER_RIGHT_MIN", "1981")
    right_max = os.getenv("GRIPPER_RIGHT_MAX", "3069")

    gripper_node = Node(
        package="xtrainer_gripper",
        executable="gripper_node",
        name=ns,
        namespace=ns,
        output="screen",
        parameters=[
            {"left_port": left_port},
            {"left_servo_id": int(left_id)},
            {"left_servo_min_pos": int(left_min)},
            {"left_servo_max_pos": int(left_max)},
            {"right_port": right_port},
            {"right_servo_id": int(right_id)},
            {"right_servo_min_pos": int(right_min)},
            {"right_servo_max_pos": int(right_max)},
            {"publish_rate": 20.0},
            {"torque_limit": 300},
        ],
    )

    return LaunchDescription([gripper_node])
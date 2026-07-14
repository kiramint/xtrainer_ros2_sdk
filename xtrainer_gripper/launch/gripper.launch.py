import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 从环境变量或默认值获取参数
    port = os.getenv("GRIPPER_PORT", "/dev/ttyUSB0")
    servo_id = os.getenv("GRIPPER_SERVO_ID", "1")
    servo_min = os.getenv("GRIPPER_SERVO_MIN", "2048")
    servo_max = os.getenv("GRIPPER_SERVO_MAX", "3998")

    gripper_node = Node(
        package="xtrainer_gripper",
        executable="gripper_node",
        name="gripper_node",
        output="screen",
        parameters=[
            {"port": port},
            {"servo_id": int(servo_id)},
            {"servo_min_pos": int(servo_min)},
            {"servo_max_pos": int(servo_max)},
            {"publish_rate": 20.0},
        ],
    )

    return LaunchDescription([gripper_node])
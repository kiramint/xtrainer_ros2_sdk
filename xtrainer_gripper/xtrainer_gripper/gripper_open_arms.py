#!/usr/bin/env python3
"""
一键张开双臂夹爪

用法:
    ros2 run xtrainer_gripper gripper_open_arms
    ros2 run xtrainer_gripper gripper_open_arms --ros-args -p gripper_ns:=/gripper
    ros2 run xtrainer_gripper gripper_open_arms --ros-args -p speed:=0.5
"""

import time

import rclpy
from rclpy.node import Node

from xtrainer_gripper.gripper_control import GripperController


class GripperOpenArms(Node):
    """一键张开双臂夹爪"""

    def __init__(self):
        super().__init__("gripper_open_arms")
        self.declare_parameter("gripper_ns", "/gripper")
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("wait", True)


def main():
    rclpy.init()
    node = GripperOpenArms()

    gripper_ns = node.get_parameter("gripper_ns").value
    speed = node.get_parameter("speed").value
    wait = node.get_parameter("wait").value

    controller = GripperController(node, gripper_ns)

    # 等待初始状态缓存
    for _ in range(10):
        controller.spin_once()
        time.sleep(0.05)

    node.get_logger().info(f"张开双臂夹爪 (speed={speed})")
    controller.open_both(speed=speed)

    if wait:
        node.get_logger().info("等待到位...")
        ok_left = controller.wait_status("left", "OPENED", timeout=10.0)
        ok_right = controller.wait_status("right", "OPENED", timeout=10.0)
        if ok_left and ok_right:
            node.get_logger().info("双臂夹爪已张开")
        else:
            node.get_logger().warn(
                f"超时: left={ok_left}, right={ok_right}, "
                f"status=({controller.read_status('left')}, {controller.read_status('right')})"
            )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
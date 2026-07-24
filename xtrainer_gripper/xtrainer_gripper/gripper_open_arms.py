#!/usr/bin/env python3
"""
一键张开双臂夹爪

用法:
    ros2 run xtrainer_gripper gripper_open_arms
    ros2 run xtrainer_gripper gripper_open_arms --ros-args -p gripper_ns:=/gripper
    ros2 run xtrainer_gripper gripper_open_arms --ros-args -p speed:=0.5
    ros2 run xtrainer_gripper gripper_open_arms --ros-args -p wait:=true
    ros2 run xtrainer_gripper gripper_open_arms --ros-args -p position_tolerance:=0.15
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
        self.declare_parameter("wait", False)
        self.declare_parameter("command_repeats", 3)
        self.declare_parameter("command_interval", 0.1)
        self.declare_parameter("timeout", 10.0)
        self.declare_parameter("position_tolerance", 0.1)


def main():
    rclpy.init()
    node = GripperOpenArms()

    gripper_ns = node.get_parameter("gripper_ns").value
    speed = node.get_parameter("speed").value
    wait = node.get_parameter("wait").value
    command_repeats = max(
        1, int(node.get_parameter("command_repeats").value)
    )
    command_interval = max(
        0.0, float(node.get_parameter("command_interval").value)
    )
    timeout = node.get_parameter("timeout").value
    tolerance = node.get_parameter("position_tolerance").value

    controller = GripperController(node, gripper_ns)

    # 等待初始状态缓存
    for _ in range(10):
        controller.spin_once()
        time.sleep(0.05)

    node.get_logger().info(
        f"无条件张开双臂夹爪 (speed={speed}, repeats={command_repeats})"
    )
    for index in range(command_repeats):
        controller.open_both(speed=speed)
        controller.spin_once()
        if index + 1 < command_repeats:
            time.sleep(command_interval)

    if wait:
        node.get_logger().info(
            f"等待张开到位 (position <= {tolerance:.2f})..."
        )
        ok_left = controller.wait_position(
            "left", 0.0, timeout=timeout, tolerance=tolerance
        )
        ok_right = controller.wait_position(
            "right", 0.0, timeout=timeout, tolerance=tolerance
        )
        if ok_left and ok_right:
            node.get_logger().info("双臂夹爪已张开")
        else:
            node.get_logger().warn(
                f"超时: left={ok_left}, right={ok_right}, "
                f"position=({controller.read_position('left'):.3f}, "
                f"{controller.read_position('right'):.3f}), "
                f"status=({controller.read_status('left')}, "
                f"{controller.read_status('right')})"
            )
    else:
        node.get_logger().info(
            "张开命令已发送，不等待 OPENED/MOVING 状态"
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
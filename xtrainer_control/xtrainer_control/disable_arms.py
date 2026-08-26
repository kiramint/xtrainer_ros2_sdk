#!/usr/bin/env python3
"""
一键去使能双臂脚本 — 可作为 ROS2 Node 运行

Usage:
    ros2 run xtrainer_control disable_arms        # 默认去使能 Arm1, Arm2
    ros2 run xtrainer_control disable_arms --ros-args -p namespaces:="['Arm1']"
"""
import sys

import rclpy
from rclpy.node import Node

from xtrainer_control.robot_control import RobotController


class DisableNode(Node):
    def __init__(self):
        super().__init__("disable_arms")
        self.declare_parameter("namespaces", ["Arm1", "Arm2"])
        namespaces = self.get_parameter("namespaces").get_parameter_value().string_array_value
        timeout = 30.0

        ctrl = RobotController(self)
        self.get_logger().info(f"Disabling arms: {namespaces}")
        ok = ctrl.disable_all(namespaces=namespaces, timeout=timeout)
        if ok:
            self.get_logger().info("All arms disabled successfully")
        else:
            self.get_logger().error("Some arms failed to disable!")
        sys.exit(0 if ok else 1)


def main():
    rclpy.init(args=sys.argv)
    node = DisableNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

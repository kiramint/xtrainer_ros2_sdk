#!/usr/bin/env python3
"""
一键使能双臂脚本 — 可作为 ROS2 Node 运行

Usage:
    ros2 run xtrainer_control enable_arms        # 默认使能 Arm1, Arm2
    ros2 run xtrainer_control enable_arms --ros-args -p namespaces:="['Arm1']"
"""
import sys

import rclpy
from rclpy.node import Node

from xtrainer_control.robot_control import enable_all


class EnableNode(Node):
    def __init__(self):
        super().__init__("enable_arms")
        self.declare_parameter("namespaces", ["Arm1", "Arm2"])
        namespaces = self.get_parameter("namespaces").get_parameter_value().string_array_value
        timeout = 120.0

        self.get_logger().info(f"Enabling arms: {namespaces}")
        ok = enable_all(self, namespaces=namespaces, timeout=timeout)
        if ok:
            self.get_logger().info("All arms enabled successfully ✓")
        else:
            self.get_logger().error("Some arms failed to enable!")
        sys.exit(0 if ok else 1)


def main():
    rclpy.init(args=sys.argv)
    node = EnableNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
一键清除双臂报警脚本。

Usage:
    ros2 run xtrainer_control clear_error       # 默认 Arm1, Arm2
    ros2 run xtrainer_control clear_error --ros-args -p namespaces:="['Arm1']"
"""
import sys

import rclpy
from rclpy.node import Node

from xtrainer_control.robot_control import clear_error_arm


class ClearErrorNode(Node):
    def __init__(self):
        super().__init__("clear_error")
        self.declare_parameter("namespaces", ["Arm1", "Arm2"])
        namespaces = self.get_parameter("namespaces").get_parameter_value().string_array_value

        all_ok = True
        for ns in namespaces:
            if not clear_error_arm(self, ns):
                all_ok = False
        if all_ok:
            self.get_logger().info("All arms: clear error OK ✓")
        else:
            self.get_logger().error("Some arms failed to clear error!")
        sys.exit(0 if all_ok else 1)


def main():
    rclpy.init(args=sys.argv)
    node = ClearErrorNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
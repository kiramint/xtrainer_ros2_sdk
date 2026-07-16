#!/usr/bin/env python3
"""
使能 + 拖拽示教双臂脚本。

Usage:
    ros2 run xtrainer_control enable_and_drag        # 默认 Arm1, Arm2
    ros2 run xtrainer_control enable_and_drag --ros-args -p namespaces:="['Arm1']"
"""
import sys

import rclpy
from rclpy.node import Node

from xtrainer_control.robot_control import enable_arm, start_drag


class EnableAndDragNode(Node):
    def __init__(self):
        super().__init__("enable_and_drag")
        self.declare_parameter("namespaces", ["Arm1", "Arm2"])
        namespaces = self.get_parameter("namespaces").get_parameter_value().string_array_value

        all_ok = True
        for ns in namespaces:
            if not enable_arm(self, ns):
                all_ok = False
                continue
            self.get_logger().info(f"[{ns}] Opening drag teaching mode ...")
            if not start_drag(self, ns):
                all_ok = False
                continue
            self.get_logger().info(f"[{ns}] Drag teaching mode ON ✓")

        if all_ok:
            self.get_logger().info("All arms: enable + drag-teaching OK ✓")
        else:
            self.get_logger().error("Some arms failed!")
        sys.exit(0 if all_ok else 1)


def main():
    rclpy.init(args=sys.argv)
    node = EnableAndDragNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
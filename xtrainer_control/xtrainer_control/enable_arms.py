#!/usr/bin/env python3
"""
一键使能双臂脚本 — 可作为 ROS2 Node 运行

Usage:
    ros2 run xtrainer_control enable_arms        # 默认使能 Arm1, Arm2
    ros2 run xtrainer_control enable_arms --ros-args -p namespaces:="['Arm1']"
    ros2 run xtrainer_control enable_arms --ros-args -p timeout:=90.0

驱动节点启动时会自己解锁抱闸。这个脚本用于中途报警、或 auto_enable:=false 之后手动再使能。
"""
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from xtrainer_control.robot_control import RobotController


def main(args=None):
    rclpy.init(args=args)
    node = Node("enable_arms")
    node.declare_parameter("namespaces", ["Arm1", "Arm2"])
    node.declare_parameter("timeout", 90.0)
    namespaces = list(node.get_parameter("namespaces").value or [])
    if not namespaces:
        namespaces = ["Arm1", "Arm2"]
    timeout = float(node.get_parameter("timeout").value)

    # 先让 executor 转起来再发服务。Jazzy 的 wait_for_service 自己不 spin,
    # 图事件没人处理时服务会一直显示 not available, 脚本直接失败。
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ctrl = RobotController(node)
        node.get_logger().info(f"Enabling arms: {list(namespaces)} (timeout={timeout}s each)")
        ok = ctrl.enable_all(namespaces=list(namespaces), timeout=timeout)
        if ok:
            node.get_logger().info("All arms enabled successfully")
        else:
            node.get_logger().error("Some arms failed to enable")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

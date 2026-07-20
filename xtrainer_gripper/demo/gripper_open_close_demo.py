#!/usr/bin/env python3
"""
夹爪开合 Demo

功能:
  演示夹爪的基本开合操作 —— 张开 → 闭合 → 张开，循环 3 次。
  通过发布 ~/left/command 或 ~/right/command 话题控制夹爪，订阅 ~/left/state 等获取反馈。

用法:
  # 测试左夹爪 (默认)
  ros2 run xtrainer_gripper gripper_open_close_demo

  # 测试右夹爪
  ros2 run xtrainer_gripper gripper_open_close_demo --ros-args -p gripper_side:=right

  # 指定命名空间
  ros2 run xtrainer_gripper gripper_open_close_demo --ros-args -p gripper_ns:=/gripper

依赖:
  - 先启动 xtrainer_gripper 节点: ros2 launch xtrainer_gripper gripper.launch.py
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class GripperOpenCloseDemo(Node):
    """夹爪开合演示节点"""

    def __init__(self):
        super().__init__("gripper_open_close_demo")

        self.declare_parameter("gripper_ns", "/gripper")
        self.declare_parameter("gripper_side", "left")
        gripper_ns = self.get_parameter("gripper_ns").value
        side = self.get_parameter("gripper_side").value

        topic_prefix = f"{gripper_ns}/{side}"

        self._cmd_pub = self.create_publisher(
            Float32MultiArray, f"{topic_prefix}/command", 10
        )
        self._status_sub = self.create_subscription(
            String, f"{topic_prefix}/status", self._status_callback, 10
        )
        self._state_sub = self.create_subscription(
            Float32MultiArray, f"{topic_prefix}/state", self._state_callback, 10
        )

        self._current_status = "UNKNOWN"
        self._current_state = [0.0, 0.0]

        self.get_logger().info(
            f"夹爪开合 Demo 启动 (目标: {topic_prefix})"
        )

    def _status_callback(self, msg: String):
        self._current_status = msg.data

    def _state_callback(self, msg: Float32MultiArray):
        self._current_state = list(msg.data)

    def _publish_cmd(self, position: float, speed: float = 1.0):
        """发布位置命令"""
        msg = Float32MultiArray()
        msg.data = [float(position), float(speed)]
        self._cmd_pub.publish(msg)

    def open(self, speed: float = 1.0, wait: bool = True):
        """张开夹爪 (0.0 = 完全张开)"""
        self.get_logger().info("⬆  张开夹爪...")
        self._publish_cmd(0.0, speed)
        if wait:
            self._wait_until_status("OPENED")

    def close(self, speed: float = 1.0, wait: bool = True):
        """闭合夹爪 (1.0 = 完全闭合)"""
        self.get_logger().info("⬇  闭合夹爪...")
        self._publish_cmd(1.0, speed)
        if wait:
            self._wait_until_status("CLOSED")

    def _wait_until_status(self, target: str, timeout: float = 10.0):
        """等待夹爪到达目标状态"""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._current_status == target:
                self.get_logger().info(
                    f"  ✓ 状态 = {self._current_status}, "
                    f"位置 = {self._current_state[0]:.3f}, "
                    f"负载 = {self._current_state[1]:.3f}"
                )
                return
        self.get_logger().warn(
            f"  ⚠ 超时: status={self._current_status}, pos={self._current_state[0]:.3f}"
        )


def main():
    rclpy.init()
    demo = GripperOpenCloseDemo()

    cycles = 3
    for i in range(cycles):
        demo.get_logger().info(f"\n{'=' * 40}\n  第 {i + 1}/{cycles} 次循环\n{'=' * 40}")
        demo.open(speed=0.8)
        time.sleep(1.0)
        demo.close(speed=0.8)
        time.sleep(1.0)

    demo.get_logger().info("\n演示完成，夹爪保持在张开状态")
    demo.open(speed=0.8)
    time.sleep(0.5)
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
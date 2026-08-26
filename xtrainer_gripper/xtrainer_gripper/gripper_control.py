#!/usr/bin/env python3
"""
XTrainer 夹爪控制库

通过 ROS2 topic 控制夹爪。与 gripper_node 通过 topic 通信，不直接访问硬件。

用法:
    from xtrainer_gripper.gripper_control import GripperController

    controller = GripperController(node, gripper_ns="/gripper")

    # 控制
    controller.open("left")
    controller.close("right")
    controller.open_both()
    controller.close_both()

    # 力矩
    controller.set_torque("left", 500)
    controller.set_torque_both(300)

    # 读取
    pos = controller.read_position("left")
    load = controller.read_load("right")
    raw_pos = controller.read_raw_position("left")
    raw_load = controller.read_raw_load("right")
    status = controller.read_status("left")
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String


class GripperController:
    """夹爪控制类 — 通过 topic 与 gripper_node 通信"""

    def __init__(
        self,
        node: Node,
        gripper_ns: str = "/gripper",
        wait_timeout: float = 2.0,
    ):
        """
        Args:
            node:          ROS2 Node 实例
            gripper_ns:    夹爪节点命名空间 (默认 /gripper)
            wait_timeout:  发送命令前等待 gripper_node 订阅者匹配的超时 (秒)。
                           避免短生命周期节点在 DDS 发现完成前 publish 导致丢消息。
        """
        self._node = node
        self._ns = gripper_ns
        self._wait_timeout = max(0.0, float(wait_timeout))
        self._cb_group = MutuallyExclusiveCallbackGroup()

        # 每侧发布器
        self._cmd_pubs = {
            "left":  node.create_publisher(Float32MultiArray, f"{gripper_ns}/left/command", 10),
            "right": node.create_publisher(Float32MultiArray, f"{gripper_ns}/right/command", 10),
        }
        self._torque_pubs = {
            "left":  node.create_publisher(Int32, f"{gripper_ns}/left/torque", 10),
            "right": node.create_publisher(Int32, f"{gripper_ns}/right/torque", 10),
        }

        # 状态缓存
        self._lock = threading.Lock()
        self._positions = {"left": -1.0, "right": -1.0}
        self._loads = {"left": -1.0, "right": -1.0}
        self._raw_positions = {"left": -1, "right": -1}
        self._raw_loads = {"left": -1, "right": -1}
        self._statuses = {"left": "UNKNOWN", "right": "UNKNOWN"}

        # 订阅
        for side in ("left", "right"):
            node.create_subscription(
                Float32MultiArray, f"{gripper_ns}/{side}/state",
                lambda msg, s=side: self._state_callback(s, msg), 10,
                callback_group=self._cb_group,
            )
            node.create_subscription(
                Int32, f"{gripper_ns}/{side}/position",
                lambda msg, s=side: self._raw_pos_callback(s, msg), 10,
                callback_group=self._cb_group,
            )
            node.create_subscription(
                Int32, f"{gripper_ns}/{side}/load_raw",
                lambda msg, s=side: self._raw_load_callback(s, msg), 10,
                callback_group=self._cb_group,
            )
            node.create_subscription(
                String, f"{gripper_ns}/{side}/status",
                lambda msg, s=side: self._status_callback(s, msg), 10,
                callback_group=self._cb_group,
            )

    # ═══════════════════════════════════════════════════════════════
    #  回调
    # ═══════════════════════════════════════════════════════════════

    def _state_callback(self, side: str, msg: Float32MultiArray):
        data = list(msg.data)
        with self._lock:
            self._positions[side] = data[0] if len(data) > 0 else -1.0
            self._loads[side] = data[1] if len(data) > 1 else -1.0

    def _raw_pos_callback(self, side: str, msg: Int32):
        with self._lock:
            self._raw_positions[side] = msg.data

    def _raw_load_callback(self, side: str, msg: Int32):
        with self._lock:
            self._raw_loads[side] = msg.data

    def _status_callback(self, side: str, msg: String):
        with self._lock:
            self._statuses[side] = msg.data

    # ═══════════════════════════════════════════════════════════════
    #  运动控制
    # ═══════════════════════════════════════════════════════════════

    def _spin_a_bit(self, timeout_sec: float):
        """spin 一次以推进 DDS 发现。

        注意: Jazzy 的 Executor.add_node 没有"已挂其他 executor"的保护,
        rclpy.spin_once 对已挂外部 executor 的 node 不会抛异常, 而是静默把
        node 再挂到 global executor 上 → 与外部 executor 并发 spin 同一
        node, callback 双重分发。因此 node 已挂 executor 时直接 sleep,
        DDS 发现由外部 executor 推进。
        """
        if getattr(self._node, "executor", None) is not None:
            time.sleep(timeout_sec)
            return
        try:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)
        except Exception:
            time.sleep(timeout_sec)

    def _ensure_subscriber(self, side: str) -> bool:
        """确保对应侧 command publisher 已与 gripper_node 订阅匹配。

        匹配后 get_subscription_count() 即时返回 >0, 无额外开销;
        仅在尚未匹配时才做有界等待。
        """
        pub = self._cmd_pubs[side]
        if pub.get_subscription_count() > 0:
            return True
        deadline = time.monotonic() + self._wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_a_bit(0.05)
            if pub.get_subscription_count() > 0:
                return True
        return False

    def _send_cmd(self, side: str, position: float, speed: float = 1.0):
        """发布位置命令 (0.0=开, 1.0=关)"""
        if not self._ensure_subscriber(side):
            self._node.get_logger().warn(
                f"{side}/command 无订阅者 (gripper_node 未启动或未发现), 仍尝试发送"
            )
        msg = Float32MultiArray()
        msg.data = [float(position), float(speed)]
        self._cmd_pubs[side].publish(msg)

    def open(self, side: str, speed: float = 1.0):
        """张开指定侧夹爪"""
        self._node.get_logger().info(f"张开 {side} 夹爪")
        self._send_cmd(side, 0.0, speed)

    def close(self, side: str, speed: float = 1.0):
        """闭合指定侧夹爪"""
        self._node.get_logger().info(f"闭合 {side} 夹爪")
        self._send_cmd(side, 1.0, speed)

    def open_both(self, speed: float = 1.0):
        """同时张开双臂夹爪"""
        self._node.get_logger().info("张开双臂夹爪")
        self._send_cmd("left", 0.0, speed)
        self._send_cmd("right", 0.0, speed)

    def close_both(self, speed: float = 1.0):
        """同时闭合双臂夹爪"""
        self._node.get_logger().info("闭合双臂夹爪")
        self._send_cmd("left", 1.0, speed)
        self._send_cmd("right", 1.0, speed)

    # ═══════════════════════════════════════════════════════════════
    #  力矩控制
    # ═══════════════════════════════════════════════════════════════

    def set_torque(self, side: str, limit: int):
        """设定指定侧力矩限制 (0~1000)"""
        limit = max(0, min(1000, limit))
        self._node.get_logger().info(f"设定 {side} 力矩限制: {limit}")
        msg = Int32(data=limit)
        self._torque_pubs[side].publish(msg)

    def set_torque_both(self, limit: int):
        """设定双臂力矩限制"""
        limit = max(0, min(1000, limit))
        self._node.get_logger().info(f"设定双臂力矩限制: {limit}")
        msg = Int32(data=limit)
        self._torque_pubs["left"].publish(msg)
        self._torque_pubs["right"].publish(msg)

    # ═══════════════════════════════════════════════════════════════
    #  读取 (非阻塞，返回最新缓存值)
    # ═══════════════════════════════════════════════════════════════

    def read_position(self, side: str) -> float:
        """读取归一化位置 (0.0~1.0, -1.0 表示无数据)"""
        with self._lock:
            return self._positions[side]

    def read_load(self, side: str) -> float:
        """读取归一化负载 (0.0~1.0, -1.0 表示无数据)"""
        with self._lock:
            return self._loads[side]

    def read_raw_position(self, side: str) -> int:
        """读取原始位置值 (0~4095, -1 表示无数据)"""
        with self._lock:
            return self._raw_positions[side]

    def read_raw_load(self, side: str) -> int:
        """读取原始负载值 (0~1000, -1 表示无数据)"""
        with self._lock:
            return self._raw_loads[side]

    def read_status(self, side: str) -> str:
        """读取状态字符串 (OPENED/CLOSED/MOVING/ERROR/UNKNOWN)"""
        with self._lock:
            return self._statuses[side]

    # ═══════════════════════════════════════════════════════════════
    #  等待
    # ═══════════════════════════════════════════════════════════════

    def position_reached(
        self,
        side: str,
        target: float,
        tolerance: float = 0.1,
    ) -> bool:
        """判断夹爪是否在目标位置容差范围内。

        Args:
            side:      ``"left"`` 或 ``"right"``。
            target:    归一化目标位置，0.0=张开，1.0=闭合。
            tolerance: 可接受的归一化位置误差。
        """
        if side not in self._positions:
            raise ValueError(f"未知夹爪侧: {side}")
        target = max(0.0, min(1.0, float(target)))
        tolerance = max(0.0, min(1.0, float(tolerance)))
        position = self.read_position(side)
        return position >= 0.0 and abs(position - target) <= tolerance

    def wait_position(
        self,
        side: str,
        target: float,
        timeout: float = 10.0,
        tolerance: float = 0.1,
    ) -> bool:
        """阻塞等待夹爪进入目标位置容差范围。"""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self.position_reached(side, target, tolerance):
                return True
        return False

    def wait_status(self, side: str, target: str, timeout: float = 10.0) -> bool:
        """阻塞等待直到夹爪到达目标状态

        Returns:
            True=到达, False=超时
        """
        deadline = time.monotonic() + max(0.0, float(timeout))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            with self._lock:
                if self._statuses[side] == target:
                    return True
        return False

    def spin_once(self):
        """更新一次状态缓存"""
        rclpy.spin_once(self._node, timeout_sec=0.05)
#!/usr/bin/env python3
"""
xtrainer_gripper: 独立夹爪 ROS2 驱动节点

通过 USB-485 控制 Feetech SMS/STS 系列舵机夹爪。
与其他模块 (dobot_bringup, xtrainer_bridge) 完全解耦。

话题:
  /gripper/command       (std_msgs/Float32MultiArray)  — [position, speed]
  /gripper/state         (std_msgs/Float32MultiArray)  — [position, current_raw]
  /gripper/status        (std_msgs/String)             — "OPENED" / "CLOSED" / "MOVING"

参数:
  port          — 串口设备 (默认 /dev/ttyUSB0)
  servo_id      — 舵机 ID (默认 1)
  servo_min_pos — 舵机内部最小位置 (默认 2048)
  servo_max_pos — 舵机内部最大位置 (默认 3998)
  publish_rate  — 状态发布频率 Hz (默认 20)
"""

import os
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .scservo_sdk import (SMS_STS_PRESENT_LOAD_L, PortHandler,
                          protocol_packet_handler, sms_sts)


def _set_latency_timer(port_path: str) -> None:
    """降低 USB-485 适配器的延迟定时器 (Linux 专有)"""
    name = os.path.basename(port_path)
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    if os.path.exists(path):
        try:
            subprocess.run(["sudo", "chmod", "666", path], check=False, capture_output=True)
            with open(path, "w") as f:
                f.write("1")
        except (PermissionError, FileNotFoundError):
            pass  # 非 root 或无文件时静默


class GripperNode(Node):
    """夹爪 ROS2 驱动节点"""

    def __init__(self):
        super().__init__("gripper_node")

        # ── 参数 ──
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("servo_min_pos", 2048)
        self.declare_parameter("servo_max_pos", 3998)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("torque_limit", 0.5)

        port = self.get_parameter("port").value
        self._servo_id = self.get_parameter("servo_id").value
        self._servo_min = self.get_parameter("servo_min_pos").value
        self._servo_max = self.get_parameter("servo_max_pos").value
        pub_rate = self.get_parameter("publish_rate").value
        torque_limit = self.get_parameter("torque_limit").value

        # 对外接口: 0.0 = 完全张开, 1.0 = 完全闭合
        self._OUT_MIN = 0.0
        self._OUT_MAX = 1.0

        # ── 初始化舵机 ──
        _set_latency_timer(port)
        self._port_handler = PortHandler(port)
        self._pack_handler = protocol_packet_handler(self._port_handler, 0)
        self._servo = sms_sts(self._port_handler)
        self._port_handler.setBaudRate(1000000)

        # Torque Enable
        self._pack_handler.write1ByteTxRx(self._servo_id, 40, 1)

        self.get_logger().info(
            f"夹爪初始化完成 port={port}, id={self._servo_id}, "
            f"servo_range=[{self._servo_min}, {self._servo_max}]"
        )

        # ── 话题 ──
        self._cmd_sub = self.create_subscription(
            Float32MultiArray, "/gripper/command", self._cmd_callback, 10
        )
        self._state_pub = self.create_publisher(Float32MultiArray, "/gripper/state", 10)
        self._status_pub = self.create_publisher(String, "/gripper/status", 10)

        # ── 定时发布状态 ──
        self._timer = self.create_timer(1.0 / pub_rate, self._publish_state)

        # ── 状态缓存 ──
        self._target_position = 0.0

    # ═══════════════════════════════════════════════════════════════
    #  命令回调
    # ═══════════════════════════════════════════════════════════════

    def _cmd_callback(self, msg: Float32MultiArray):
        """接收 [position, speed] 命令"""
        data = list(msg.data)
        if len(data) < 1:
            return
        position = max(self._OUT_MIN, min(self._OUT_MAX, float(data[0])))
        speed = max(0.0, min(1.0, float(data[1]))) if len(data) >= 2 else 1.0
        self._target_position = position
        self._move(position, speed)

    # ═══════════════════════════════════════════════════════════════
    #  舵机通信
    # ═══════════════════════════════════════════════════════════════

    def _normalized_to_servo(self, val: float) -> int:
        return int(val * (self._servo_max - self._servo_min) + self._servo_min)

    def _servo_to_normalized(self, raw: int) -> float:
        return (raw - self._servo_min) / (self._servo_max - self._servo_min)

    def _move(self, position: float, speed: float):
        """移动夹爪到指定位置"""
        target_pos = self._normalized_to_servo(position)
        target_speed = int(speed * 4095)  # 0~4095
        result, error = self._servo.WritePosEx(
            self._servo_id, target_pos, target_speed, 0
        )
        if result != 0 or error != 0:
            self.get_logger().error(
                f"WritePosEx failed: result={result}, error={error}"
            )

    def _read_position(self) -> float:
        """读取当前位置 (归一化 0~1)"""
        raw, result, error = self._servo.ReadPos(self._servo_id)
        if result != 0 or error != 0:
            return -1.0
        return self._servo_to_normalized(raw)

    def _read_load(self) -> float:
        """读取当前负载 (归一化 0~1), 用于力反馈估算
        SMS_STS_PRESENT_LOAD_L = 60, 负载值范围 0~1000 (对应 0%~100% 额定力矩)
        """
        raw, result, error = self._pack_handler.read2ByteTxRx(
            self._servo_id, SMS_STS_PRESENT_LOAD_L
        )
        if result != 0 or error != 0:
            return -1.0
        return raw / 1000.0  # 归一化到 0~1

    def _publish_state(self):
        """定时发布状态"""
        pos = self._read_position()
        load = self._read_load()

        state_msg = Float32MultiArray()
        state_msg.data = [float(pos), float(load)]
        self._state_pub.publish(state_msg)

        # 状态字符串
        if pos < 0.0:
            status = "ERROR"
        elif pos < 0.05:
            status = "OPENED"
        elif pos > 0.95:
            status = "CLOSED"
        else:
            status = "MOVING"
        self._status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
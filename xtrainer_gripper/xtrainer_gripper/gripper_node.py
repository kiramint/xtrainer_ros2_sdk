#!/usr/bin/env python3
"""
xtrainer_gripper: 双臂夹爪 ROS2 驱动节点 (单节点管理左右两个夹爪)

通过 USB-485 控制 Feetech SMS/STS 系列舵机夹爪。
一个节点打开两个串口 (/dev/ttyUSB0, /dev/ttyUSB1)，分别驱动左右夹爪。

话题 (使用节点命名空间隔离，如 /gripper/left/command):
  ~/left/command   (std_msgs/Float32MultiArray)  — [position, speed]
  ~/left/state     (std_msgs/Float32MultiArray)  — [position, load]
  ~/left/status    (std_msgs/String)             — "OPENED" / "CLOSED" / "MOVING"
  ~/left/position  (std_msgs/Int32)              — 原始位置读数
  ~/left/torque    (std_msgs/Int32)              — 运行时设定力矩限制 (0~1000)
  ~/left/load_raw  (std_msgs/Int32)              — 原始负载读数 (0~1000)
  ~/right/xxx      ... (同上)

参数:
  left_port          — 左夹爪串口 (默认 /dev/ttyUSB0)
  left_servo_id      — 左夹爪舵机 ID (默认 21)
  left_servo_min_pos — 左夹爪最小位置 (默认 2500)
  left_servo_max_pos — 左夹爪最大位置 (默认 3800)
  right_port         — 右夹爪串口 (默认 /dev/ttyUSB1)
  right_servo_id     — 右夹爪舵机 ID (默认 22)
  right_servo_min_pos — 右夹爪最小位置 (默认 2500)
  right_servo_max_pos — 右夹爪最大位置 (默认 3800)
  publish_rate       — 状态发布频率 Hz (默认 20)
  torque_limit       — 力矩限制 0~1000 (默认 300)
"""

import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String

from .scservo_sdk import (SMS_STS_PRESENT_LOAD_L, PortHandler,
                          protocol_packet_handler, sms_sts)


def _set_latency_timer(port_path: str) -> None:
    """降低 USB-485 适配器的延迟定时器 (Linux 专用)

    直接尝试写入 sysfs。若无权限则静默跳过——可通过 udev 规则授权：
      SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
    """
    name = os.path.basename(port_path)
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    if os.path.exists(path):
        try:
            with open(path, "w") as f:
                f.write("1")
        except PermissionError:
            pass


class _GripperDriver:
    """单个夹爪的硬件驱动 + 话题"""

    def __init__(
        self,
        node: Node,
        side: str,
        port: str,
        servo_id: int,
        servo_min: int,
        servo_max: int,
        pub_rate: float,
        torque_limit: int,
    ):
        self._node = node
        self._side = side  # "left" or "right"
        self._servo_id = servo_id
        self._servo_min = servo_min
        self._servo_max = servo_max
        self._cmd_lock = threading.Lock()
        self._latest_cmd = None  # (position, speed)

        self._node.get_logger().info(
            f"[{side}] 初始化 port={port}, id={servo_id}, range=[{servo_min}, {servo_max}]"
        )

        # ── 串口与舵机初始化 ──
        _set_latency_timer(port)
        self._port_handler = PortHandler(port)
        self._pack_handler = protocol_packet_handler(self._port_handler, 0)
        self._servo = sms_sts(self._port_handler)
        self._port_handler.setBaudRate(1000000)

        # Ping
        model_number, result, error = self._servo.ping(servo_id)
        if error == 0 and result == 0:
            self._node.get_logger().info(f"[{side}] Ping OK: ID={servo_id}, model={model_number}")
        else:
            self._node.get_logger().error(
                f"[{side}] Ping 失败: result={result}, error={error}. "
                f"请检查串口 {port} 和舵机 ID {servo_id}"
            )

        # Torque Enable
        ft_comm_result, ft_error = self._pack_handler.write1ByteTxRx(servo_id, 40, 1)
        if ft_comm_result != 0 or ft_error != 0:
            self._node.get_logger().warn(
                f"[{side}] Torque Enable 失败: comm_result={ft_comm_result}, error={ft_error}"
            )

        # 力矩限制
        self._torque_limit = torque_limit
        self._set_torque_hw(torque_limit)
        self._node.get_logger().info(f"[{side}] 力矩限制已设置: {torque_limit}")

        # ── 话题 ──
        self._cmd_sub = node.create_subscription(
            Float32MultiArray, f"{side}/command", self._cmd_callback, 10
        )
        self._torque_sub = node.create_subscription(
            Int32, f"{side}/torque", self._torque_callback, 10
        )
        self._state_pub = node.create_publisher(Float32MultiArray, f"{side}/state", 10)
        self._status_pub = node.create_publisher(String, f"{side}/status", 10)
        self._raw_pos_pub = node.create_publisher(Int32, f"{side}/position", 10)
        self._raw_load_pub = node.create_publisher(Int32, f"{side}/load_raw", 10)

        # ── 定时器 ──
        self._state_timer = node.create_timer(1.0 / pub_rate, self._publish_state)
        self._cmd_timer = node.create_timer(0.03, self._process_command)

    # ═══════════════════════════════════════════════════════════════
    #  命令
    # ═══════════════════════════════════════════════════════════════

    def _cmd_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 1:
            return
        position = max(0.0, min(1.0, float(data[0])))
        speed = max(0.0, min(1.0, float(data[1]))) if len(data) >= 2 else 1.0
        self._latest_cmd = (position, speed)

    def _torque_callback(self, msg: Int32):
        val = max(0, min(1000, msg.data))
        self._set_torque_hw(val)
        self._torque_limit = val

    def _process_command(self):
        if self._latest_cmd is None:
            return
        position, speed = self._latest_cmd
        self._latest_cmd = None
        self._move(position, speed)

    # ═══════════════════════════════════════════════════════════════
    #  舵机通信
    # ═══════════════════════════════════════════════════════════════

    def _set_torque_hw(self, limit: int):
        with self._cmd_lock:
            result, error = self._pack_handler.write2ByteTxRx(
                self._servo_id, 48, limit
            )
        if result != 0 or error != 0:
            self._node.get_logger().error(
                f"[{self._side}] 力矩限制设置失败: result={result}, error={error}"
            )

    def _normalized_to_servo(self, val: float) -> int:
        # 0.0=张开→_servo_max, 1.0=闭合→_servo_min
        return int(self._servo_max - val * (self._servo_max - self._servo_min))

    def _servo_to_normalized(self, raw: int) -> float:
        # _servo_max=张开→0.0, _servo_min=闭合→1.0
        return (self._servo_max - raw) / (self._servo_max - self._servo_min)

    def _move(self, position: float, speed: float):
        target_pos = self._normalized_to_servo(position)
        target_speed = int(speed * 4095)
        with self._cmd_lock:
            result, error = self._servo.WritePosEx(
                self._servo_id, target_pos, target_speed, 0
            )
        if result != 0 or error != 0:
            self._node.get_logger().error(
                f"[{self._side}] WritePosEx failed: result={result}, error={error}"
            )

    def _read_position(self) -> float:
        with self._cmd_lock:
            raw, result, error = self._servo.ReadPos(self._servo_id)
        if result != 0 or error != 0:
            self._node.get_logger().debug(
                f"[{self._side}] ReadPos failed: result={result}, error={error}",
                throttle_duration_sec=5.0,
            )
            return -1.0
        return self._servo_to_normalized(raw)

    def _read_load(self) -> float:
        with self._cmd_lock:
            raw, result, error = self._pack_handler.read2ByteTxRx(
                self._servo_id, SMS_STS_PRESENT_LOAD_L
            )
        if result != 0 or error != 0:
            self._node.get_logger().debug(
                f"[{self._side}] ReadLoad failed: result={result}, error={error}",
                throttle_duration_sec=5.0,
            )
            return -1.0
        return raw / 1000.0

    def _read_raw_position(self) -> int:
        with self._cmd_lock:
            raw, result, error = self._servo.ReadPos(self._servo_id)
        if result != 0 or error != 0:
            return -1
        return raw

    def _read_raw_load(self) -> int:
        with self._cmd_lock:
            raw, result, error = self._pack_handler.read2ByteTxRx(
                self._servo_id, SMS_STS_PRESENT_LOAD_L
            )
        if result != 0 or error != 0:
            return -1
        return raw

    def _publish_state(self):
        raw_pos = self._read_raw_position()
        raw_load = self._read_raw_load()
        pos = self._read_position()
        load = self._read_load()

        state_msg = Float32MultiArray()
        state_msg.data = [float(pos), float(load)]
        self._state_pub.publish(state_msg)
        self._raw_pos_pub.publish(Int32(data=raw_pos))
        self._raw_load_pub.publish(Int32(data=raw_load))

        if pos < 0.0:
            status = "ERROR"
        elif pos < 0.05:
            status = "OPENED"
        elif pos > 0.95:
            status = "CLOSED"
        else:
            status = "MOVING"
        self._status_pub.publish(String(data=status))


class GripperNode(Node):
    """双臂夹爪 ROS2 驱动节点 (管理左右两个夹爪)"""

    _SIDE_PARAMS = [
        ("left",  "/dev/ttyUSB0", 21, 2500, 3800),
        ("right", "/dev/ttyUSB1", 22, 2500, 3800),
    ]

    def __init__(self):
        super().__init__("gripper_node")

        # 通用参数
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("torque_limit", 300)

        # 每侧参数
        for side, default_port, default_id, default_min, default_max in self._SIDE_PARAMS:
            self.declare_parameter(f"{side}_port", default_port)
            self.declare_parameter(f"{side}_servo_id", default_id)
            self.declare_parameter(f"{side}_servo_min_pos", default_min)
            self.declare_parameter(f"{side}_servo_max_pos", default_max)

        pub_rate = self.get_parameter("publish_rate").value
        torque_limit = self.get_parameter("torque_limit").value

        # 创建左右夹爪驱动
        self._left = _GripperDriver(
            self, "left",
            port=self.get_parameter("left_port").value,
            servo_id=self.get_parameter("left_servo_id").value,
            servo_min=self.get_parameter("left_servo_min_pos").value,
            servo_max=self.get_parameter("left_servo_max_pos").value,
            pub_rate=pub_rate,
            torque_limit=torque_limit,
        )
        self._right = _GripperDriver(
            self, "right",
            port=self.get_parameter("right_port").value,
            servo_id=self.get_parameter("right_servo_id").value,
            servo_min=self.get_parameter("right_servo_min_pos").value,
            servo_max=self.get_parameter("right_servo_max_pos").value,
            pub_rate=pub_rate,
            torque_limit=torque_limit,
        )

        self.get_logger().info("双臂夹爪节点初始化完成")


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
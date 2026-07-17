#!/usr/bin/env python3
"""
舵机 ID 扫描工具

扫描串口上所有可能的舵机 ID (0~253)，找到能成功 Ping 的舵机。

用法:
  python3 xtrainer_gripper/demo/scan_servo_id.py                    # 默认 /dev/ttyUSB0
  python3 xtrainer_gripper/demo/scan_servo_id.py /dev/ttyUSB1       # 指定串口
"""

import os
import subprocess
import sys


def set_latency_timer(port_path: str) -> None:
    name = os.path.basename(port_path)
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    if os.path.exists(path):
        try:
            subprocess.run(["sudo", "chmod", "666", path], check=False, capture_output=True)
            with open(path, "w") as f:
                f.write("1")
        except (PermissionError, FileNotFoundError):
            pass


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node("scan_servo_id", start_parameter_services=False)

    set_latency_timer(port)

    # 导入 SDK（使用 xtrainer_gripper 内嵌的 scservo_sdk）
    from xtrainer_gripper.scservo_sdk import (PortHandler,
                                              protocol_packet_handler, sms_sts)

    port_handler = PortHandler(port)
    pack_handler = protocol_packet_handler(port_handler, 0)
    servo = sms_sts(port_handler)

    if not port_handler.setBaudRate(1000000):
        node.get_logger().error(f"无法打开串口: {port}")
        return

    node.get_logger().info(f"正在扫描 {port} (波特率 1000000)...")
    node.get_logger().info("=" * 50)

    found = []
    # 扫描 ID 0~253 (SMS/STS 默认 ID=1, 广播地址=0xFE 跳过)
    for scs_id in range(0, 254):
        # 跳过广播地址
        if scs_id == 0xFE:
            continue
        model_number, result, error = servo.ping(scs_id)
        if error == 0 and result == 0:
            found.append((scs_id, model_number))
            node.get_logger().info(f"  ✓ ID={scs_id:3d}  model={model_number}")

    node.get_logger().info("=" * 50)
    if found:
        node.get_logger().info(f"找到 {len(found)} 个舵机:")
        for sid, model in found:
            node.get_logger().info(f"  ID={sid}  model={model}")
    else:
        node.get_logger().warn("未找到任何舵机!")
        node.get_logger().warn("请检查:")
        node.get_logger().warn(f"  1. 串口 {port} 是否正确")
        node.get_logger().warn("  2. 舵机是否已通电（24V 电源）")
        node.get_logger().warn("  3. RS485 接线 A/B 是否对应")

    port_handler.closePort()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
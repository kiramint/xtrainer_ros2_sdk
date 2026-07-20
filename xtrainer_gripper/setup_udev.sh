#!/bin/bash
#
# 设置 USB-485 适配器延迟定时器 udev 规则
#
# 将 ftdi_sio 驱动的 latency_timer 设为 1ms，降低串口通信延迟。
# 运行后无需 sudo，夹爪节点会自动写入 sysfs 生效。
#
# 用法:
#   sudo ./setup_udev.sh
#

set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-xtrainer-gripper-latency.rules"
RULE_CONTENT='SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"'

if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 此脚本需 root 权限运行 (sudo ./setup_udev.sh)"
    exit 1
fi

echo "→ 写入 udev 规则: $RULE_FILE"
echo "$RULE_CONTENT" > "$RULE_FILE"
chmod 644 "$RULE_FILE"

echo "→ 重载 udev 规则"
udevadm control --reload-rules

echo "→ 触发已连接设备"
udevadm trigger --subsystem-match=usb-serial

echo "✓ 完成. 规则内容:"
cat "$RULE_FILE"
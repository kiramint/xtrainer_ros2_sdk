#!/usr/bin/env python3
"""
夹爪恒力夹取 Demo

功能:
  演示基于负载反馈的"伪恒力"夹取策略。
  夹爪以缓慢速度闭合，当检测到负载超过阈值时停止，
  如果物体滑脱（负载下降），自动补夹。

用法:
  # 默认左夹爪
  ros2 run xtrainer_gripper gripper_constant_force_demo

  # 指定右夹爪
  ros2 run xtrainer_gripper gripper_constant_force_demo --ros-args -p gripper_side:=right

  # 指定命名空间
  ros2 run xtrainer_gripper gripper_constant_force_demo --ros-args -p gripper_ns:=/gripper

工作原理:
  1. 夹爪全开
  2. 缓慢闭合 (speed≈0.3)
  3. 实时监控 ~/left/state 中的 load 值
  4. 当 load > force_threshold 时 → 停止闭合 (物体已夹住)
  5. 保持监控：若 load 下降 (物体滑脱) → 自动补夹半步
  6. 用户 Ctrl+C 退出，夹爪自动张开释放

依赖:
  - 先启动 xtrainer_gripper 节点: ros2 launch xtrainer_gripper gripper.launch.py

参数 (可修改):
  force_threshold — 负载阈值, 0.0~1.0 (默认 0.15, 即额定力矩的 15%)
  close_speed     — 夹取速度, 0.0~1.0 (默认 0.3)
  hold_duration   — 保持时间 (秒), 默认 5.0
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

# ═══════════════════════════════════════════════════════════════
#  可调参数
# ═══════════════════════════════════════════════════════════════
FORCE_THRESHOLD = 0.18   # 负载阈值 (0~1), 超过此值认为已夹住物体
CLOSE_SPEED = 0.3        # 闭合速度 (0~1), 越小越精细
SLIP_DETECT_THRESHOLD = 0.05  # 负载下降超过此值判定为滑脱
RETRY_STEP = 0.08        # 补夹步长 (0~1)
HOLD_DURATION = 5.0      # 保持夹持时间 (秒)


class ConstantForceDemo(Node):
    """恒力夹取演示节点"""

    def __init__(self):
        super().__init__("constant_force_demo")

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

        self._current_position = 0.0
        self._current_load = 0.0
        self._current_status = "UNKNOWN"
        self._holding = False
        self._held_load = 0.0

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"  恒力夹取 Demo 启动 (目标: {topic_prefix})")
        self.get_logger().info(f"  阈值: {FORCE_THRESHOLD:.2f}  速度: {CLOSE_SPEED:.2f}")
        self.get_logger().info("=" * 50)

    def _status_callback(self, msg: String):
        self._current_status = msg.data

    def _state_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        self._current_position = data[0] if len(data) > 0 else 0.0
        self._current_load = data[1] if len(data) > 1 else 0.0

    def _publish_cmd(self, position: float, speed: float = 1.0):
        msg = Float32MultiArray()
        msg.data = [max(0.0, min(1.0, float(position))), float(speed)]
        self._cmd_pub.publish(msg)

    def _spin_and_log(self, timeout_sec: float = 0.1):
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def open(self):
        """完全张开"""
        self.get_logger().info("⬆  张开夹爪...")
        self._publish_cmd(0.0, 1.0)
        start = time.time()
        while time.time() - start < 5.0:
            self._spin_and_log()
            if self._current_status == "OPENED":
                self.get_logger().info("  ✓ 已张开")
                return
        self.get_logger().warn("  ⚠ 张开超时")

    def close_until_force(self) -> bool:
        """缓慢闭合直到检测到负载超过阈值"""
        self.get_logger().info("⬇  缓慢闭合中... (监测负载)")
        step_size = 0.02
        current_target = 0.0

        while current_target < 1.0:
            current_target += step_size
            current_target = min(current_target, 1.0)
            self._publish_cmd(current_target, CLOSE_SPEED)

            # 等待位置稳定 + 读取负载
            for _ in range(15):  # ~1.5s per step at 10Hz
                self._spin_and_log(0.1)
                if self._current_load >= FORCE_THRESHOLD:
                    self.get_logger().info(
                        f"  ✓ 检测到物体! "
                        f"load={self._current_load:.3f} ≥ {FORCE_THRESHOLD:.3f}, "
                        f"pos={self._current_position:.3f}"
                    )
                    self._holding = True
                    self._held_load = self._current_load
                    return True

            self._spin_and_log(0.1)

        self.get_logger().warn("  ⚠ 夹爪已完全闭合，未检测到物体")
        return False

    def monitor_slip(self) -> bool:
        """监控滑脱并自动补夹"""
        start = time.time()
        slip_count = 0

        while time.time() - start < HOLD_DURATION:
            self._spin_and_log(0.2)

            if self._current_load < self._held_load - SLIP_DETECT_THRESHOLD:
                slip_count += 1
                self.get_logger().warn(
                    f"  ⚡ 检测到滑脱 #{slip_count}! "
                    f"load {self._current_load:.3f} < {self._held_load - SLIP_DETECT_THRESHOLD:.3f}"
                )

                # 补夹
                new_target = min(1.0, self._current_position + RETRY_STEP)
                self.get_logger().info(f"  → 补夹到 pos={new_target:.3f}")
                self._publish_cmd(new_target, CLOSE_SPEED * 0.5)
                time.sleep(0.5)

                # 更新参考负载
                self._held_load = self._current_load

            if slip_count >= 3:
                self.get_logger().error("  ✗ 滑脱次数过多，放弃")
                return False

        return True

    def run(self):
        """执行一次恒力夹取流程"""
        try:
            # 1. 张开
            self.open()

            # 2. 缓慢闭合直到检测到负载
            if not self.close_until_force():
                self.get_logger().warn("夹取失败 (未检测到物体)")
                return False

            # 3. 保持并监控滑脱
            self.get_logger().info(
                f"\n🔄 保持夹持中... (持续 {HOLD_DURATION}s, 监控滑脱)"
            )
            if not self.monitor_slip():
                return False

            self.get_logger().info(
                f"\n✓ 恒力夹取成功! "
                f"最终 load={self._current_load:.3f}, pos={self._current_position:.3f}"
            )
            return True

        except KeyboardInterrupt:
            self.get_logger().info("\n中断! 张开夹爪释放物体...")
            return False
        finally:
            self.open()


def main():
    rclpy.init()
    demo = ConstantForceDemo()

    try:
        success = demo.run()
        if success:
            demo.get_logger().info("\n演示完成!")
        else:
            demo.get_logger().warn("\n演示未成功")
    except KeyboardInterrupt:
        demo.get_logger().info("\n用户中断")
    finally:
        demo.open()
        time.sleep(0.5)
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
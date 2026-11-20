#!/usr/bin/env python3
"""
XTrainer 机械臂状态控制库

提供 RobotController 类，封装 PowerOn + EnableRobot 的时序逻辑。
node 由外部传入，使用独立的 MutuallyExclusiveCallbackGroup 避免与主 executor 竞争。

依赖: dobot_msgs_v4 自定义服务消息
"""

import re
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from dobot_msgs_v4.srv import (BrakeControl, ClearError, DisableRobot, EnableRobot,
                               PowerOn, RobotMode, StartDrag, StopDrag)

# RobotMode 返回值常量 (Dobot 协议)
ROBOT_MODE_INIT = 1          # 初始化
ROBOT_MODE_BRAKE_OPEN = 2    # 有抱闸松开
ROBOT_MODE_NO_POWER = 3      # 本体未上电
ROBOT_MODE_DISABLED = 4      # 未使能 (可调用 EnableRobot)
ROBOT_MODE_IDLE = 5          # 使能空闲
ROBOT_MODE_DRAG = 6          # 拖拽示教模式
ROBOT_MODE_RUNNING = 7       # 运行中
ROBOT_MODE_SINGLE_MOVE = 8   # 点动 / RunTo
ROBOT_MODE_ERROR = 9         # 有未清除的报警
ROBOT_MODE_PAUSE = 10        # 暂停
ROBOT_MODE_COLLISION = 11    # 碰撞

# 伺服已使能、抱闸由控制器松开
ENABLED_MODES = {
    ROBOT_MODE_IDLE,
    ROBOT_MODE_DRAG,
    ROBOT_MODE_RUNNING,
    ROBOT_MODE_SINGLE_MOVE,
    ROBOT_MODE_PAUSE,
}
# 已下电或未使能, 抱闸处于锁定
BRAKED_MODES = {ROBOT_MODE_NO_POWER, ROBOT_MODE_DISABLED}


class RobotController:
    """
    机械臂状态控制器。

    使用独立的 MutuallyExclusiveCallbackGroup 执行服务调用，
    避免与其他回调（如 timer、topic）在 MultiThreadedExecutor 中竞争。

    用法::

        node = Node('my_node')
        ctrl = RobotController(node)
        ctrl.enable_arm('Arm1')
        ctrl.disable_all()
    """

    def __init__(self, node: Node):
        self._node = node
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._clients = {}

    # ── 内部工具 ──────────────────────────────────────────────────

    def _get_client(self, srv_type, service_name: str):
        """按 (服务类型, 服务名) 缓存 client, 避免 create_client 泄漏。"""
        key = (srv_type.__name__, service_name)
        client = self._clients.get(key)
        if client is None:
            client = self._node.create_client(
                srv_type, service_name, callback_group=self._cb_group
            )
            self._clients[key] = client
        return client

    def _owns_executor(self) -> bool:
        return getattr(self._node, "executor", None) is not None

    def _pump_discovery(self) -> None:
        """节点还没挂到 executor 时, wait_for_service 自己不 spin, 图事件到不了。"""
        if self._owns_executor():
            return
        try:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            return

    def _wait_future(self, future, timeout: float) -> bool:
        """等待 future 完成, 返回是否成功完成。

        节点已经属于某个 executor 时只轮询 future.done()。
        再调 spin_until_future_complete 会把同一个 node 挂进 global executor,
        和常驻 executor 并发 spin, 相机等高频回调被双重分发。
        """
        if self._owns_executor():
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() >= deadline or not rclpy.ok():
                    return False
                time.sleep(0.005)
            return True
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        return future.done()

    def _call_service(
        self, srv_type, service_name: str, timeout: float = 5.0, request=None
    ) -> Tuple[bool, int]:
        """
        通用 ROS2 服务调用，带超时和就绪等待。

        Returns:
            (success, res_code): success 为 True 表示传输成功, res_code 为控制器 ErrorID。
            ErrorID 为 0 才表示控制器接受了指令。
        """
        client = self._get_client(srv_type, service_name)

        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.1):
            self._pump_discovery()
            if time.monotonic() >= deadline:
                self._node.get_logger().error(
                    f"Service {service_name} not available after {timeout}s"
                )
                return False, -1
            if not rclpy.ok():
                return False, -1

        req = request if request is not None else srv_type.Request()
        future = client.call_async(req)
        remaining = max(0.1, deadline - time.monotonic())
        if not self._wait_future(future, remaining):
            self._node.get_logger().error(f"Service call {service_name} timed out")
            return False, -1

        res = future.result()
        if res is None:
            return False, -1
        return True, int(res.res)

    # ── 查询 ──────────────────────────────────────────────────────

    def get_robot_mode(self, arm_namespace: str, timeout: float = 5.0) -> int:
        """
        获取机械臂当前模式 (RobotMode)。

        Returns:
            mode 值: 3=本体未上电, 4=未使能, 5=使能空闲, 6=拖拽模式, 7=运行中, 9=报错
            -1 表示获取失败
        """
        ns = arm_namespace.rstrip("/")
        srv_name = f"/{ns}/dobot_bringup_ros2/srv/RobotMode"

        client = self._get_client(RobotMode, srv_name)
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.1):
            self._pump_discovery()
            if time.monotonic() >= deadline or not rclpy.ok():
                self._node.get_logger().warning(f"RobotMode service not available for {ns}")
                return -1

        req = RobotMode.Request()
        future = client.call_async(req)
        remaining = max(0.1, deadline - time.monotonic())
        if not self._wait_future(future, remaining):
            self._node.get_logger().error(f"RobotMode call timed out for {ns}")
            return -1

        res = future.result()
        if res is None:
            return -1
        # 控制器返回形如 "0,{5},RobotMode();", 驱动把花括号里的值放进 robot_return。
        match = re.search(r"-?\d+", getattr(res, "robot_return", "") or "")
        if match is None:
            return -1
        return int(match.group(0))

    # ── 单臂操作 ──────────────────────────────────────────────────

    def clear_error_arm(self, arm_namespace: str = "Arm1", timeout: float = 5.0) -> bool:
        """清除指定机械臂的报警 (ClearError)。"""
        ns = arm_namespace.rstrip("/")
        srv_name = f"/{ns}/dobot_bringup_ros2/srv/ClearError"

        self._node.get_logger().info(f"[{ns}] ClearError ...")
        ok, res = self._call_service(ClearError, srv_name, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] ClearError failed")
            return False
        self._node.get_logger().info(f"[{ns}] ClearError OK (res={res})")
        return True

    def _remaining(self, deadline: float) -> float:
        return deadline - time.monotonic()

    def _wait(self, seconds: float, deadline: float) -> bool:
        """睡一小段, 返回是否还没到总截止时间。"""
        end = min(deadline, time.monotonic() + seconds)
        while time.monotonic() < end:
            if not rclpy.ok():
                return False
            time.sleep(0.05)
        return self._remaining(deadline) > 0

    def _call_until(self, srv_type, service_name: str, deadline: float, request=None):
        call_timeout = min(8.0, max(0.5, self._remaining(deadline)))
        return self._call_service(srv_type, service_name, call_timeout, request)

    def enable_arm(self, arm_namespace: str = "Arm1", timeout: float = 90.0) -> bool:
        """
        使能指定机械臂, 直到 RobotMode 进入已使能, 或超时。

        按实时模式决定下一步, 而不是固定走一遍 PowerOn:
        - 5/6/7/8/10: 已经松开抱闸, 直接成功
        - 9/11: ClearError 后重试
        - 0/1 或查询失败: 控制柜还没就绪, 继续等
        - 3: PowerOn, 然后等上电完成 (约 10s), 期间不重复下发
        - 2: 有关节抱闸被手动松开, 先逐轴锁上
        - 4: EnableRobot, 并确认模式真的变成已使能

        控制器拒绝某一步 (ErrorID != 0) 或 Dashboard 暂时连不上时继续重试,
        直到 timeout。
        """
        ns = arm_namespace.rstrip("/")
        clearerror_srv = f"/{ns}/dobot_bringup_ros2/srv/ClearError"
        poweron_srv = f"/{ns}/dobot_bringup_ros2/srv/PowerOn"
        enable_srv = f"/{ns}/dobot_bringup_ros2/srv/EnableRobot"
        brake_srv = f"/{ns}/dobot_bringup_ros2/srv/BrakeControl"
        deadline = time.monotonic() + timeout
        last_mode = None
        log = self._node.get_logger()

        while self._remaining(deadline) > 0 and rclpy.ok():
            mode = self.get_robot_mode(ns, timeout=min(3.0, max(0.5, self._remaining(deadline))))
            if mode != last_mode:
                log.info(f"[{ns}] RobotMode={mode}")
                last_mode = mode

            if mode in ENABLED_MODES:
                log.info(f"[{ns}] brakes released (mode={mode})")
                return True

            if mode in (ROBOT_MODE_ERROR, ROBOT_MODE_COLLISION):
                ok, res = self._call_until(ClearError, clearerror_srv, deadline)
                log.info(f"[{ns}] ClearError ok={ok} res={res}")
                if not self._wait(0.5, deadline):
                    break
                continue

            if mode < 0 or mode == ROBOT_MODE_INIT:
                if not self._wait(0.5, deadline):
                    break
                continue

            if mode == ROBOT_MODE_NO_POWER:
                ok, res = self._call_until(PowerOn, poweron_srv, deadline)
                log.info(f"[{ns}] PowerOn ok={ok} res={res}")
                power_deadline = min(deadline, time.monotonic() + 15.0)
                while time.monotonic() < power_deadline and rclpy.ok():
                    mode = self.get_robot_mode(
                        ns, timeout=min(3.0, max(0.5, self._remaining(deadline)))
                    )
                    if mode not in (-1, 0, ROBOT_MODE_INIT, ROBOT_MODE_NO_POWER):
                        last_mode = mode
                        log.info(f"[{ns}] RobotMode={mode}")
                        break
                    if not self._wait(0.5, power_deadline):
                        break
                continue

            if mode == ROBOT_MODE_BRAKE_OPEN:
                for axis in range(1, 7):
                    if self._remaining(deadline) <= 0:
                        break
                    req = BrakeControl.Request()
                    req.axis_id = axis
                    req.value = 0
                    ok, res = self._call_until(BrakeControl, brake_srv, deadline, req)
                    log.info(f"[{ns}] BrakeControl({axis},0) ok={ok} res={res}")
                if not self._wait(0.3, deadline):
                    break
                continue

            if mode == ROBOT_MODE_DISABLED:
                ok, res = self._call_until(EnableRobot, enable_srv, deadline)
                log.info(f"[{ns}] EnableRobot ok={ok} res={res}")
                confirmed = False
                for _ in range(15):
                    if self._remaining(deadline) <= 0 or not rclpy.ok():
                        break
                    mode = self.get_robot_mode(
                        ns, timeout=min(3.0, max(0.5, self._remaining(deadline)))
                    )
                    if mode in ENABLED_MODES:
                        log.info(f"[{ns}] brakes released (mode={mode})")
                        return True
                    if mode in (
                        ROBOT_MODE_BRAKE_OPEN,
                        ROBOT_MODE_ERROR,
                        ROBOT_MODE_COLLISION,
                    ):
                        last_mode = mode
                        confirmed = True
                        break
                    if not self._wait(0.2, deadline):
                        break
                if confirmed:
                    continue
                if not self._wait(0.5, deadline):
                    break
                continue

            if not self._wait(0.5, deadline):
                break

        log.error(f"[{ns}] enable timed out after {timeout}s, last mode={last_mode}")
        return False

    def disable_arm(self, arm_namespace: str = "Arm1", timeout: float = 10.0) -> bool:
        """去使能指定机械臂: DisableRobot。已经抱闸 (mode 3/4) 也算成功。"""
        ns = arm_namespace.rstrip("/")
        disable_srv = f"/{ns}/dobot_bringup_ros2/srv/DisableRobot"
        log = self._node.get_logger()

        mode = self.get_robot_mode(ns, timeout=min(3.0, timeout))
        if mode in BRAKED_MODES:
            log.info(f"[{ns}] already braked (mode={mode})")
            return True
        # 拖拽模式下直接 DisableRobot 会被拒绝, 先退出示教。
        if mode == ROBOT_MODE_DRAG:
            self.stop_drag(ns, timeout=min(5.0, timeout))

        log.info(f"[{ns}] DisableRobot ...")
        ok, res = self._call_service(DisableRobot, disable_srv, timeout)
        if ok and res == 0:
            log.info(f"[{ns}] DisableRobot OK (res={res})")
            return True

        mode = self.get_robot_mode(ns, timeout=min(3.0, timeout))
        if mode in BRAKED_MODES:
            log.info(f"[{ns}] already braked (mode={mode})")
            return True
        log.error(f"[{ns}] DisableRobot failed (ok={ok} res={res} mode={mode})")
        return False

    def start_drag(self, arm_namespace: str = "Arm1", timeout: float = 5.0) -> bool:
        """开启指定机械臂的拖拽示教模式 (StartDrag)。"""
        ns = arm_namespace.rstrip("/")
        srv_name = f"/{ns}/dobot_bringup_ros2/srv/StartDrag"

        self._node.get_logger().info(f"[{ns}] StartDrag ...")
        ok, res = self._call_service(StartDrag, srv_name, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] StartDrag failed")
            return False
        self._node.get_logger().info(
            f"[{ns}] StartDrag OK (res={res}), now in teaching mode"
        )
        return True

    def stop_drag(self, arm_namespace: str = "Arm1", timeout: float = 5.0) -> bool:
        """关闭指定机械臂的拖拽示教模式 (StopDrag)。"""
        ns = arm_namespace.rstrip("/")
        srv_name = f"/{ns}/dobot_bringup_ros2/srv/StopDrag"

        self._node.get_logger().info(f"[{ns}] StopDrag ...")
        ok, res = self._call_service(StopDrag, srv_name, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] StopDrag failed")
            return False
        self._node.get_logger().info(f"[{ns}] StopDrag OK (res={res})")
        return True

    # ── 多臂操作 ──────────────────────────────────────────────────

    def enable_all(
        self,
        namespaces: Optional[List[str]] = None,
        timeout: float = 90.0,
    ) -> bool:
        """使能所有指定机械臂，默认 ['Arm1', 'Arm2']。timeout 是每条臂各自的上限。"""
        if namespaces is None:
            namespaces = ["Arm1", "Arm2"]
        all_ok = True
        for ns in namespaces:
            if not self.enable_arm(ns, timeout):
                all_ok = False
        return all_ok

    def disable_all(
        self,
        namespaces: Optional[List[str]] = None,
        timeout: float = 10.0,
    ) -> bool:
        """去使能所有指定机械臂，默认 ['Arm1', 'Arm2']。"""
        if namespaces is None:
            namespaces = ["Arm1", "Arm2"]
        all_ok = True
        for ns in namespaces:
            if not self.disable_arm(ns, timeout):
                all_ok = False
        return all_ok
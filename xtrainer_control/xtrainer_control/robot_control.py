#!/usr/bin/env python3
"""
XTrainer 机械臂状态控制库

提供 RobotController 类，封装 PowerOn + EnableRobot 的时序逻辑。
node 由外部传入，使用独立的 MutuallyExclusiveCallbackGroup 避免与主 executor 竞争。

依赖: dobot_msgs_v4 自定义服务消息
"""

import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from dobot_msgs_v4.srv import (ClearError, DisableRobot, EnableRobot, PowerOn,
                               RobotMode, StartDrag, StopDrag)

# RobotMode 返回值常量 (Dobot 协议)
ROBOT_MODE_INIT = 1          # 初始化
ROBOT_MODE_BRAKE_OPEN = 2    # 有抱闸松开
ROBOT_MODE_NO_POWER = 3      # 本体未上电
ROBOT_MODE_DISABLED = 4      # 未使能 (可调用 EnableRobot)
ROBOT_MODE_IDLE = 5          # 使能空闲
ROBOT_MODE_DRAG = 6          # 拖拽示教模式
ROBOT_MODE_RUNNING = 7       # 运行中
ROBOT_MODE_ERROR = 9         # 有未清除的报警


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

    def _wait_future(self, future, timeout: float) -> bool:
        """等待 future 完成, 返回是否成功完成。

        若 node 已挂载到外部 executor (如 MultiThreadedExecutor 常驻 spin),
        直接轮询 future.done(), 响应回调由常驻 executor 处理。
        绝不能再调 rclpy.spin_until_future_complete —— 它会把 node 再挂到
        global executor 与常驻 executor 并发 spin, 相机等高频回调被双重
        分发、时序打乱 (表现为相机延迟骤增)。

        若 node 未挂载任何 executor (独立 CLI 节点), 回退到自旋等待。
        """
        executor = getattr(self._node, "executor", None)
        if executor is not None and getattr(executor, "is_spinning", False):
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() >= deadline or not rclpy.ok():
                    return False
                time.sleep(0.005)
            return future.done()
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        return future.done()

    def _call_service(
        self, srv_type, service_name: str, timeout: float = 5.0
    ) -> Tuple[bool, int]:
        """
        通用 ROS2 服务调用，带超时和就绪等待。

        Returns:
            (success, res_code): success 为 True 表示调用成功, res_code 为服务返回码
        """
        client = self._get_client(srv_type, service_name)

        # 等待服务就绪
        start = time.time()
        while not client.wait_for_service(timeout_sec=0.1):
            if time.time() - start > timeout:
                self._node.get_logger().error(
                    f"Service {service_name} not available after {timeout}s"
                )
                return False, -1
            if not rclpy.ok():
                return False, -1

        req = srv_type.Request()
        future = client.call_async(req)

        if not self._wait_future(future, timeout):
            self._node.get_logger().error(f"Service call {service_name} timed out")
            return False, -1

        res = future.result()
        return True, res.res

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
        if not client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().error(f"RobotMode service not available for {ns}")
            return -1

        req = RobotMode.Request()
        future = client.call_async(req)

        if not self._wait_future(future, timeout):
            self._node.get_logger().error(f"RobotMode call timed out for {ns}")
            return -1

        res = future.result()
        # robot_return 格式: "{5}" 或 "{3}"
        try:
            mode = int(res.robot_return.strip("{}"))
            return mode
        except (ValueError, AttributeError):
            return -1

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

    def enable_arm(self, arm_namespace: str = "Arm1", timeout: float = 120.0) -> bool:
        """
        使能指定机械臂: ClearError → PowerOn → 等待上电完成 → EnableRobot。

        Dobot 协议要求 PowerOn 后需约 10 秒完成初始化，
        通过轮询 RobotMode 来精确等待。
        """
        ns = arm_namespace.rstrip("/")
        clearerror_srv = f"/{ns}/dobot_bringup_ros2/srv/ClearError"
        poweron_srv = f"/{ns}/dobot_bringup_ros2/srv/PowerOn"
        enable_srv = f"/{ns}/dobot_bringup_ros2/srv/EnableRobot"

        # ── Step 0: 检查是否已经使能 ──
        mode = self.get_robot_mode(ns)
        self._node.get_logger().info(f"[{ns}] Initial mode: {mode}")
        if mode >= ROBOT_MODE_IDLE and mode != ROBOT_MODE_ERROR:
            self._node.get_logger().info(
                f"[{ns}] Already enabled (mode={mode}), skipping"
            )
            return True

        # ── Step 1: ClearError (清除上次的报警) ──
        self._node.get_logger().info(f"[{ns}] Step 1/4: ClearError ...")
        ok, res = self._call_service(ClearError, clearerror_srv, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] ClearError failed")
        else:
            self._node.get_logger().info(f"[{ns}] ClearError OK (res={res})")

        # ── Step 2: PowerOn ──
        self._node.get_logger().info(f"[{ns}] Step 2/4: PowerOn ...")
        ok, res = self._call_service(PowerOn, poweron_srv, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] PowerOn failed")
            return False
        self._node.get_logger().info(f"[{ns}] PowerOn OK (res={res})")

        # ── Step 3: Wait for power-on to complete (mode >= 4) ──
        self._node.get_logger().info(
            f"[{ns}] Step 3/4: Waiting for power-on to complete ..."
        )
        start = time.time()
        while time.time() - start < timeout:
            mode = self.get_robot_mode(ns)
            self._node.get_logger().info(f"[{ns}] Current mode: {mode}")
            if mode == ROBOT_MODE_ERROR:
                self._node.get_logger().error(
                    f"[{ns}] Arm entered error mode (9), aborting"
                )
                return False
            if mode >= ROBOT_MODE_DISABLED:
                break
            time.sleep(1.0)
        else:
            self._node.get_logger().error(
                f"[{ns}] Power-on wait timed out after {timeout}s"
            )
            return False

        self._node.get_logger().info(f"[{ns}] Power-on complete, mode={mode}")

        # ── Step 4: EnableRobot ──
        self._node.get_logger().info(f"[{ns}] Step 4/4: EnableRobot ...")
        ok, res = self._call_service(EnableRobot, enable_srv, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] EnableRobot failed (res={res})")
            return False
        self._node.get_logger().info(f"[{ns}] EnableRobot OK (res={res})")

        self._node.get_logger().info(f"[{ns}] Enable complete")
        return True

    def disable_arm(self, arm_namespace: str = "Arm1", timeout: float = 10.0) -> bool:
        """去使能指定机械臂: DisableRobot。"""
        ns = arm_namespace.rstrip("/")
        disable_srv = f"/{ns}/dobot_bringup_ros2/srv/DisableRobot"

        self._node.get_logger().info(f"[{ns}] DisableRobot ...")
        ok, res = self._call_service(DisableRobot, disable_srv, timeout)
        if not ok:
            self._node.get_logger().error(f"[{ns}] DisableRobot failed")
            return False
        self._node.get_logger().info(f"[{ns}] DisableRobot OK (res={res})")

        self._node.get_logger().info(f"[{ns}] Disable complete")
        return True

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
        timeout: float = 10.0,
    ) -> bool:
        """使能所有指定机械臂，默认 ['Arm1', 'Arm2']。"""
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
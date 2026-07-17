#!/usr/bin/env python3
"""
XTrainer 机械臂状态控制库

提供 enable_arm / disable_arm 函数，封装 PowerOn + EnableRobot 的时序逻辑。

依赖: dobot_msgs_v4 自定义服务消息
"""

import time
from typing import List, Tuple

import rclpy
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


def _call_service(node: Node, srv_type, service_name: str, timeout: float = 5.0) -> Tuple[bool, int]:
    """
    通用 ROS2 服务调用，带超时和就绪等待。

    Args:
        node:       ROS2 Node 实例
        srv_type:   服务类型 (EnableRobot / DisableRobot / PowerOn)
        service_name: 完整服务路径，如 '/Arm1/dobot_bringup_ros2/srv/EnableRobot'
        timeout:    等待服务就绪的超时时间 (秒)

    Returns:
        (success, res_code): success 为 True 表示调用成功, res_code 为服务返回码
    """
    client = node.create_client(srv_type, service_name)

    # 等待服务就绪
    start = time.time()
    while not client.wait_for_service(timeout_sec=0.1):
        if time.time() - start > timeout:
            node.get_logger().error(f"Service {service_name} not available after {timeout}s")
            return False, -1
        if not rclpy.ok():
            return False, -1

    req = srv_type.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    if future.result() is None:
        node.get_logger().error(f"Service call {service_name} timed out")
        return False, -1

    res = future.result()
    return True, res.res


def _get_robot_mode(node: Node, arm_namespace: str, timeout: float = 5.0) -> int:
    """
    获取机械臂当前模式 (RobotMode)。

    Returns:
        mode 值: 3=本体未上电, 4=未使能, 5=使能空闲, 6=拖拽模式, 7=运行中, 9=报错
        -1 表示获取失败
    """
    ns = arm_namespace.rstrip("/")
    srv_name = f"/{ns}/dobot_bringup_ros2/srv/RobotMode"

    client = node.create_client(RobotMode, srv_name)
    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(f"RobotMode service not available for {ns}")
        return -1

    req = RobotMode.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)

    if future.result() is None:
        node.get_logger().error(f"RobotMode call timed out for {ns}")
        return -1

    res = future.result()
    # robot_return 格式: "{5}" 或 "{3}"
    try:
        mode = int(res.robot_return.strip("{}"))
        return mode
    except (ValueError, AttributeError):
        return -1


def clear_error_arm(node: Node, arm_namespace: str = "Arm1", timeout: float = 5.0) -> bool:
    """
    清除指定机械臂的报警 (ClearError)。

    Args:
        node:           ROS2 Node 实例
        arm_namespace:  机械臂命名空间，如 'Arm1', 'Arm2'
        timeout:        超时时间 (秒)

    Returns:
        是否成功
    """
    ns = arm_namespace.rstrip("/")
    srv_name = f"/{ns}/dobot_bringup_ros2/srv/ClearError"

    node.get_logger().info(f"[{ns}] ClearError ...")
    ok, res = _call_service(node, ClearError, srv_name, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] ClearError failed")
        return False
    node.get_logger().info(f"[{ns}] ClearError OK (res={res})")
    return True


def enable_arm(node: Node, arm_namespace: str = "Arm1", timeout: float = 30.0) -> bool:
    """
    使能指定机械臂: ClearError → PowerOn → 等待上电完成 → EnableRobot。

    Dobot 协议要求 PowerOn 后需约 10 秒完成初始化，
    通过轮询 RobotMode 来精确等待。

    Args:
        node:           ROS2 Node 实例
        arm_namespace:  机械臂命名空间，如 'Arm1', 'Arm2'
        timeout:        总超时时间 (秒)，默认 30s

    Returns:
        是否成功使能
    """
    ns = arm_namespace.rstrip("/")
    clearerror_srv = f"/{ns}/dobot_bringup_ros2/srv/ClearError"
    poweron_srv = f"/{ns}/dobot_bringup_ros2/srv/PowerOn"
    enable_srv = f"/{ns}/dobot_bringup_ros2/srv/EnableRobot"

    # ── Step 0: 检查是否已经使能 ──
    mode = _get_robot_mode(node, ns)
    node.get_logger().info(f"[{ns}] Initial mode: {mode}")
    if mode >= ROBOT_MODE_IDLE:  # 5=使能空闲, 6=拖拽, 7=运行中
        node.get_logger().info(f"[{ns}] Already enabled (mode={mode}), skipping")
        return True

    # ── Step 1: ClearError (清除上次的报警) ──
    node.get_logger().info(f"[{ns}] Step 1/4: ClearError ...")
    ok, res = _call_service(node, ClearError, clearerror_srv, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] ClearError failed")
        # 不返回 False，ClearError 失败不影响后续流程
    else:
        node.get_logger().info(f"[{ns}] ClearError OK (res={res})")

    # ── Step 2: PowerOn ──
    node.get_logger().info(f"[{ns}] Step 2/4: PowerOn ...")
    ok, res = _call_service(node, PowerOn, poweron_srv, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] PowerOn failed")
        return False
    node.get_logger().info(f"[{ns}] PowerOn OK (res={res})")

    # ── Step 3: Wait for power-on to complete (mode ≥ 4) ──
    node.get_logger().info(f"[{ns}] Step 3/4: Waiting for power-on to complete ...")
    start = time.time()
    while time.time() - start < timeout:
        mode = _get_robot_mode(node, ns)
        node.get_logger().info(f"[{ns}] Current mode: {mode}")
        if mode == ROBOT_MODE_ERROR:
            node.get_logger().error(f"[{ns}] Arm entered error mode (9), aborting")
            return False
        if mode >= ROBOT_MODE_DISABLED:  # mode 4+
            break
        time.sleep(1.0)
    else:
        node.get_logger().error(f"[{ns}] Power-on wait timed out after {timeout}s")
        return False

    node.get_logger().info(f"[{ns}] Power-on complete, mode={mode}")

    # ── Step 4: EnableRobot ──
    node.get_logger().info(f"[{ns}] Step 4/4: EnableRobot ...")
    ok, res = _call_service(node, EnableRobot, enable_srv, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] EnableRobot failed (res={res})")
        return False
    node.get_logger().info(f"[{ns}] EnableRobot OK (res={res})")

    node.get_logger().info(f"[{ns}] Enable complete ✓")
    return True


def disable_arm(node: Node, arm_namespace: str = "Arm1", timeout: float = 10.0) -> bool:
    """
    去使能指定机械臂: DisableRobot。

    Args:
        node:           ROS2 Node 实例
        arm_namespace:  机械臂命名空间，如 'Arm1', 'Arm2'
        timeout:        总超时时间 (秒)

    Returns:
        是否成功去使能
    """
    ns = arm_namespace.rstrip("/")
    disable_srv = f"/{ns}/dobot_bringup_ros2/srv/DisableRobot"

    node.get_logger().info(f"[{ns}] DisableRobot ...")
    ok, res = _call_service(node, DisableRobot, disable_srv, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] DisableRobot failed")
        return False
    node.get_logger().info(f"[{ns}] DisableRobot OK (res={res})")

    node.get_logger().info(f"[{ns}] Disable complete ✓")
    return True


def start_drag(node: Node, arm_namespace: str = "Arm1", timeout: float = 5.0) -> bool:
    """
    开启指定机械臂的拖拽示教模式 (StartDrag)。

    Args:
        node:           ROS2 Node 实例
        arm_namespace:  机械臂命名空间，如 'Arm1', 'Arm2'
        timeout:        超时时间 (秒)

    Returns:
        是否成功
    """
    ns = arm_namespace.rstrip("/")
    srv_name = f"/{ns}/dobot_bringup_ros2/srv/StartDrag"

    node.get_logger().info(f"[{ns}] StartDrag ...")
    ok, res = _call_service(node, StartDrag, srv_name, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] StartDrag failed")
        return False
    node.get_logger().info(f"[{ns}] StartDrag OK (res={res}), now in teaching mode")
    return True


def stop_drag(node: Node, arm_namespace: str = "Arm1", timeout: float = 5.0) -> bool:
    """
    关闭指定机械臂的拖拽示教模式 (StopDrag)。

    Args:
        node:           ROS2 Node 实例
        arm_namespace:  机械臂命名空间，如 'Arm1', 'Arm2'
        timeout:        超时时间 (秒)

    Returns:
        是否成功
    """
    ns = arm_namespace.rstrip("/")
    srv_name = f"/{ns}/dobot_bringup_ros2/srv/StopDrag"

    node.get_logger().info(f"[{ns}] StopDrag ...")
    ok, res = _call_service(node, StopDrag, srv_name, timeout)
    if not ok:
        node.get_logger().error(f"[{ns}] StopDrag failed")
        return False
    node.get_logger().info(f"[{ns}] StopDrag OK (res={res})")
    return True


def enable_all(node: Node, namespaces: List[str] = None, timeout: float = 10.0) -> bool:
    """
    使能所有指定机械臂。

    Args:
        node:        ROS2 Node 实例
        namespaces:  机械臂命名空间列表，默认 ['Arm1', 'Arm2']
        timeout:     每个机械臂的超时时间 (秒)

    Returns:
        是否全部成功
    """
    if namespaces is None:
        namespaces = ["Arm1", "Arm2"]
    all_ok = True
    for ns in namespaces:
        if not enable_arm(node, ns, timeout):
            all_ok = False
    return all_ok


def disable_all(node: Node, namespaces: List[str] = None, timeout: float = 10.0) -> bool:
    """
    去使能所有指定机械臂。

    Args:
        node:        ROS2 Node 实例
        namespaces:  机械臂命名空间列表，默认 ['Arm1', 'Arm2']
        timeout:     每个机械臂的超时时间 (秒)

    Returns:
        是否全部成功
    """
    if namespaces is None:
        namespaces = ["Arm1", "Arm2"]
    all_ok = True
    for ns in namespaces:
        if not disable_arm(node, ns, timeout):
            all_ok = False
    return all_ok
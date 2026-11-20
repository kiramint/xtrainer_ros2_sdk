"""stdio MCP。日志走 stderr，stdout 只留给协议。"""

from __future__ import annotations

import logging
import os
import sys
import threading
from pathlib import Path

_vendor = Path(__file__).resolve().parents[1] / ".vendor"
if _vendor.is_dir():
    _vendor_path = str(_vendor)
    if _vendor_path not in sys.path:
        sys.path.insert(0, _vendor_path)

from mcp.server.mcpserver import MCPServer

from xtrainer_mcp.node import XTrainerMcpNode

mcp = MCPServer("xtrainer_mcp")
_node: XTrainerMcpNode | None = None


def _call(method: str, *args, **kwargs):
    if _node is None:
        return {"ok": False, "error": "MCP 节点还没连上 ROS"}
    try:
        return getattr(_node, method)(*args, **kwargs)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


@mcp.tool()
def get_camera_frame(camera: str) -> dict:
    """返回一路相机的最新彩色 JPEG 和 16 位深度 PNG（毫米），以及宽高、光学坐标系、时间戳和内参 K。

    camera 取 camera_top、camera_left、camera_right。没有画面时立刻返回错误，不会等待。
    深度 PNG 是对齐到彩色图的原始深度，0 表示无效。
    """
    return _call("get_camera_frame", camera)


@mcp.tool()
def project_pixel(camera: str, u: int, v: int, window: int = 5) -> dict:
    """把深度图上的像素变成 base_link 坐标。

    先取窗口内非零深度的中值（毫米），再用内参反投影到光学坐标系（X 右、Y 下、Z 前），
    然后按该帧时间戳查手眼标定写进 TF 的变换。window 默认 5。
    """
    return _call("project_pixel", camera, u, v, window)


@mcp.tool()
def transform_optical_point(camera: str, x: float, y: float, z: float) -> dict:
    """把光学坐标系中的一个点（米）用最新 TF 变到 base_link。"""
    return _call("transform_optical_point", camera, x, y, z)


@mcp.tool()
def get_tcp_pose(arm: str, link: str = "tcp") -> dict:
    """读取该臂 TCP 在 base_link 下的位置和姿态。link 可取 tcp 或 tip。

    关节来自 /joint_states。优先用 move_group 的 /compute_fk，服务不在时用同一份 URDF 计算。
    """
    return _call("get_tcp_pose", arm, link)


@mcp.tool()
def get_status() -> dict:
    """双臂模式、关节角、TCP、夹爪状态，以及还留着的计划编号。不驱动机器人。"""
    return _call("get_status")


@mcp.tool()
def plan_joints(arm: str, positions: list[float], planner: str = "ompl") -> dict:
    """规划到 6 个关节角（弧度，J1 到 J6 的顺序）。只规划，不运动。

    planner 取 ompl、pilz_ptp、pilz_lin。返回的 JSON 是给 agent 看的；
    超过 200 个点会抽稀，并带上原始点数。执行时用的是完整轨迹。
    """
    return _call("plan_joints", arm, positions, planner)


@mcp.tool()
def plan_pose(
    arm: str,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    link: str = "tcp",
    planner: str = "ompl",
) -> dict:
    """规划到 base_link 下的位置（米）和四元数姿态。link 可取 tcp 或 tip。只规划，不运动。"""
    return _call("plan_pose", arm, x, y, z, qx, qy, qz, qw, link, planner)


@mcp.tool()
def plan_named(arm: str, name: str, planner: str = "ompl") -> dict:
    """规划到 SRDF 里的命名姿态。Arm1 用 Home1，Arm2 用 Home2。只规划，不运动。"""
    return _call("plan_named", arm, name, planner)


@mcp.tool()
def execute_plan(plan_id: str) -> dict:
    """执行已经规划好的轨迹。会挡住直到结束，这一版不能取消。

    计划不存在、起点和当前关节差超过 0.05 rad、自碰撞，或已经有别的动作在跑，都会拒绝。
    """
    return _call("execute_plan", plan_id)


@mcp.tool()
def check_self_collision() -> dict:
    """用 move_group 检查当前整机（两条臂一起）是否自碰撞。没有点云障碍。"""
    return _call("check_self_collision")


@mcp.tool()
def check_joints(arm: str, positions: list[float]) -> dict:
    """把一条臂放到给定关节角，另一条臂保持当前角度，检查整机自碰撞。"""
    return _call("check_joints", arm, positions)


@mcp.tool()
def check_plan(plan_id: str) -> dict:
    """沿一条已缓存轨迹抽样做整机自碰撞检查，返回接触点。"""
    return _call("check_plan", plan_id)


@mcp.tool()
def fk(arm: str, positions: list[float], link: str = "tcp") -> dict:
    """对给定关节角做正运动学。优先 /compute_fk，否则用 URDF，和 get_tcp_pose 同一套模型。"""
    return _call("fk", arm, positions, link)


@mcp.tool()
def ik(
    arm: str,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    link: str = "tcp",
) -> dict:
    """逆运动学。种子是当前关节，并检查解是否自碰撞。位置在 base_link，单位米。"""
    return _call("ik", arm, x, y, z, qx, qy, qz, qw, link)


@mcp.tool()
def servo_joints(arm: str, points: list[list[float]], dt: float) -> dict:
    """按固定间隔把关节角（弧度）直接发给 ServoJ。度数换算只发生在发送前。

    最多 60 个点，dt 在 0.02 到 0.5 秒，单轴相邻步长不超过 0.15 rad，总时长不超过 5 秒。
    任何一个点自碰撞，整段都不发送。
    """
    return _call("servo_joints", arm, points, dt)


@mcp.tool()
def gripper_open(side: str, speed: float = 1.0) -> dict:
    """张开夹爪。side 取 left、right 或 both。speed 是 0 到 1。"""
    return _call("gripper_open", side, speed)


@mcp.tool()
def gripper_close(side: str, speed: float = 1.0) -> dict:
    """闭合夹爪。side 取 left、right 或 both。speed 是 0 到 1。"""
    return _call("gripper_close", side, speed)


@mcp.tool()
def gripper_set_torque(side: str, limit: int) -> dict:
    """设置夹爪力矩上限，0 到 1000。side 取 left、right 或 both。"""
    return _call("gripper_set_torque", side, limit)


@mcp.tool()
def gripper_get_state(side: str = "both") -> dict:
    """读取夹爪开合位置、负载和状态字。不驱动夹爪。"""
    return _call("gripper_get_state", side)


@mcp.tool()
def clear_error(arm: str = "both") -> dict:
    """清除报警。arm 取 Arm1、Arm2 或 both。"""
    return _call("clear_error", arm)


@mcp.tool()
def unlock_brakes(arm: str = "both") -> dict:
    """解锁抱闸，直到 RobotMode 进入 5、6、7、8 或 10。可能要等上电，最长约 90 秒。"""
    return _call("unlock_brakes", arm)


@mcp.tool()
def lock_brakes(arm: str = "both") -> dict:
    """锁定抱闸。拖拽模式会先退出示教，再 DisableRobot。"""
    return _call("lock_brakes", arm)


@mcp.tool()
def get_robot_mode(arm: str = "both") -> dict:
    """读取 RobotMode。不改变抱闸。"""
    return _call("get_robot_mode", arm)


def main() -> None:
    os.environ["RCUTILS_LOGGING_USE_STDOUT"] = "0"
    logging.basicConfig(level=logging.INFO, stream=sys.stderr, force=True)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = XTrainerMcpNode()
    global _node
    _node = node
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, name="xtrainer_mcp_spin", daemon=True)
    thread.start()
    try:
        mcp.run(transport="stdio")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

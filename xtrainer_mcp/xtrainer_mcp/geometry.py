"""纯计算：像素反投影、轨迹 JSON、URDF 正运动学、ServoJ 路径检查。

这里不导入 rclpy，单元测试可以直接跑。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Iterable, Mapping, Optional, Sequence

import numpy as np

BASE_FRAME = "base_link"
ARMS = ("Arm1", "Arm2")
ARM_JOINTS = {
    "Arm1": [f"J1_{i}" for i in range(1, 7)],
    "Arm2": [f"J2_{i}" for i in range(1, 7)],
}
ARM_LINKS = {
    "Arm1": {"tcp": "L1_gripper_tcp", "tip": "L1_gripper_tip"},
    "Arm2": {"tcp": "L2_gripper_tcp", "tip": "L2_gripper_tip"},
}
CAMERAS = ("camera_top", "camera_left", "camera_right")
OPTICAL_FRAMES = {name: f"{name}_color_optical_frame" for name in CAMERAS}

PLAN_JSON_MAX_POINTS = 200
SERVO_MAX_POINTS = 60
SERVO_DT_MIN = 0.02
SERVO_DT_MAX = 0.5
SERVO_MAX_STEP = 0.15
SERVO_MAX_DURATION = 5.0
EXECUTE_START_TOLERANCE = 0.05

PLANNERS = {
    "ompl": ("ompl", "RRTConnect"),
    "pilz_ptp": ("pilz_industrial_motion_planner", "PTP"),
    "pilz_lin": ("pilz_industrial_motion_planner", "LIN"),
}


def f6(value: float) -> float:
    return round(float(value), 6)


def require_arm(arm: str) -> str:
    if arm not in ARM_JOINTS:
        raise ValueError(f"arm 只能是 Arm1 或 Arm2，收到 {arm!r}")
    return arm


def resolve_link(arm: str, link: Optional[str]) -> str:
    """tcp / tip / 完整 link 名。省略时用该臂的 TCP。"""
    arm = require_arm(arm)
    aliases = ARM_LINKS[arm]
    if link is None or link == "" or link == "tcp":
        return aliases["tcp"]
    if link == "tip":
        return aliases["tip"]
    if link in aliases.values():
        return link
    raise ValueError(
        f"{arm} 的 link 只能是 tcp、tip、{aliases['tcp']} 或 {aliases['tip']}，收到 {link!r}"
    )


def unproject(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    """像素 + 深度 → 光学坐标系。X 右、Y 下、Z 前，单位米。"""
    if fx == 0.0 or fy == 0.0:
        raise ValueError("相机焦距为 0")
    z = float(depth_m)
    x = (float(u) - float(cx)) / float(fx) * z
    y = (float(v) - float(cy)) / float(fy) * z
    return x, y, z


def median_depth_mm(depth: np.ndarray, u: int, v: int, window: int) -> Optional[float]:
    """窗口内非零深度的中值，单位与 depth 数组一致（对齐深度图是毫米）。"""
    if window < 1:
        raise ValueError("window 至少为 1")
    u = int(u)
    v = int(v)
    if depth.ndim != 2:
        raise ValueError("深度图必须是二维数组")
    height, width = depth.shape
    if u < 0 or v < 0 or u >= width or v >= height:
        return None
    half = int(window) // 2
    patch = depth[
        max(0, v - half) : min(height, v + half + 1),
        max(0, u - half) : min(width, u + half + 1),
    ]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def select_indices(count: int, max_points: int) -> list[int]:
    """均匀抽点，始终保留第一点和最后一点。"""
    if count < 0:
        raise ValueError("count 不能为负")
    if max_points < 2:
        raise ValueError("max_points 至少为 2")
    if count <= max_points:
        return list(range(count))
    last = count - 1
    chosen: list[int] = []
    for i in range(max_points):
        idx = int(round(i * last / (max_points - 1)))
        if not chosen or chosen[-1] != idx:
            chosen.append(idx)
    if chosen[-1] != last:
        chosen.append(last)
    return chosen


def trajectory_json(
    joint_names: Sequence[str],
    times: Sequence[float],
    positions: Sequence[Sequence[float]],
    velocities: Optional[Sequence[Sequence[float]]] = None,
    max_points: int = PLAN_JSON_MAX_POINTS,
) -> dict:
    count = len(positions)
    if len(times) != count:
        raise ValueError("时间和关节位置数量不一致")
    indices = select_indices(count, max_points)
    points = []
    for idx in indices:
        vel = []
        if velocities is not None and idx < len(velocities) and velocities[idx]:
            vel = [f6(v) for v in velocities[idx]]
        points.append(
            {
                "t": f6(times[idx]),
                "positions": [f6(v) for v in positions[idx]],
                "velocities": vel,
            }
        )
    return {
        "joint_names": list(joint_names),
        "duration_sec": f6(times[-1]) if times else 0.0,
        "point_count": count,
        "returned_point_count": len(points),
        "points": points,
    }


def validate_servo_path(
    current: Sequence[float],
    points: Sequence[Sequence[float]],
    dt: float,
) -> Optional[str]:
    """返回错误说明；路径合法时返回 None。还不做碰撞检查。"""
    if not points:
        return "ServoJ 路径是空的"
    if len(points) > SERVO_MAX_POINTS:
        return f"ServoJ 最多 {SERVO_MAX_POINTS} 个点，收到 {len(points)} 个"
    if dt < SERVO_DT_MIN or dt > SERVO_DT_MAX:
        return f"dt 必须在 {SERVO_DT_MIN} 到 {SERVO_DT_MAX} 秒之间，收到 {dt}"
    if len(points) * float(dt) > SERVO_MAX_DURATION + 1e-9:
        return f"ServoJ 总时长不能超过 {SERVO_MAX_DURATION} 秒"
    width = len(current)
    if width == 0:
        return "当前关节位置为空"
    previous = [float(v) for v in current]
    for index, point in enumerate(points):
        if len(point) != width:
            return f"第 {index} 个点有 {len(point)} 个关节，当前状态有 {width} 个"
        for axis, value in enumerate(point):
            number = float(value)
            if not math.isfinite(number):
                return f"第 {index} 个点的关节 {axis} 不是有限数"
            if abs(number - previous[axis]) > SERVO_MAX_STEP:
                return (
                    f"第 {index} 个点的关节 {axis} 单步 {abs(number - previous[axis]):.4f} rad，"
                    f"超过 {SERVO_MAX_STEP} rad"
                )
        previous = [float(v) for v in point]
    return None


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _axis_rotation(axis: Iterable[float], angle: float) -> np.ndarray:
    vector = np.asarray(list(axis), dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = vector / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )


def _homogeneous(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = list(translation)
    return transform


def _xyz_rpy(text_xyz: Optional[str], text_rpy: Optional[str]) -> tuple[np.ndarray, np.ndarray]:
    xyz = [float(part) for part in (text_xyz or "0 0 0").split()]
    rpy = [float(part) for part in (text_rpy or "0 0 0").split()]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError(f"origin 无法解析: xyz={text_xyz!r} rpy={text_rpy!r}")
    return _rpy_matrix(*rpy), xyz


def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    return x, y, z, w


def pose_dict(transform: np.ndarray, frame: str = BASE_FRAME) -> dict:
    qx, qy, qz, qw = matrix_to_quaternion(transform[:3, :3])
    return {
        "frame": frame,
        "position": {
            "x": f6(transform[0, 3]),
            "y": f6(transform[1, 3]),
            "z": f6(transform[2, 3]),
        },
        "orientation": {"x": f6(qx), "y": f6(qy), "z": f6(qz), "w": f6(qw)},
    }


class UrdfKinematics:
    """用 URDF 关节原点做正运动学，作为 /compute_fk 不可用时的同一模型回退。"""

    def __init__(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        self._parent: dict[str, str] = {}
        self._joint_to_child: dict[str, dict] = {}
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            parent_link = parent.get("link")
            child_link = child.get("link")
            if not parent_link or not child_link:
                continue
            origin = joint.find("origin")
            axis = joint.find("axis")
            rotation, translation = _xyz_rpy(
                None if origin is None else origin.get("xyz"),
                None if origin is None else origin.get("rpy"),
            )
            axis_xyz = [1.0, 0.0, 0.0]
            if axis is not None and axis.get("xyz"):
                axis_xyz = [float(part) for part in axis.get("xyz", "").split()]
            self._parent[child_link] = parent_link
            self._joint_to_child[child_link] = {
                "name": joint.get("name") or "",
                "type": joint.get("type") or "fixed",
                "rotation": rotation,
                "translation": translation,
                "axis": axis_xyz,
            }

    def link_transform(
        self,
        link: str,
        joint_positions: Optional[Mapping[str, float]] = None,
    ) -> np.ndarray:
        if link == BASE_FRAME:
            return np.eye(4)
        chain = []
        current = link
        guard = 0
        while current != BASE_FRAME:
            joint = self._joint_to_child.get(current)
            if joint is None:
                raise KeyError(f"URDF 里没有从 {BASE_FRAME} 到 {link} 的关节链（停在 {current}）")
            chain.append(joint)
            current = self._parent[current]
            guard += 1
            if guard > 100:
                raise KeyError(f"{link} 的关节链有环")
        positions = joint_positions or {}
        transform = np.eye(4)
        for joint in reversed(chain):
            origin = _homogeneous(joint["rotation"], joint["translation"])
            motion = np.eye(4)
            value = float(positions.get(joint["name"], 0.0))
            kind = joint["type"]
            if kind in ("revolute", "continuous"):
                motion[:3, :3] = _axis_rotation(joint["axis"], value)
            elif kind == "prismatic":
                axis = np.asarray(joint["axis"], dtype=float)
                norm = float(np.linalg.norm(axis))
                if norm > 1e-12:
                    motion[:3, 3] = axis / norm * value
            transform = transform @ origin @ motion
        return transform

    def link_pose(
        self,
        link: str,
        joint_positions: Optional[Mapping[str, float]] = None,
    ) -> dict:
        return pose_dict(self.link_transform(link, joint_positions))


def parse_named_states(srdf_xml: str) -> dict[tuple[str, str], dict[str, float]]:
    root = ET.fromstring(srdf_xml)
    states: dict[tuple[str, str], dict[str, float]] = {}
    for state in root.findall("group_state"):
        group = state.get("group") or ""
        name = state.get("name") or ""
        joints = {}
        for joint in state.findall("joint"):
            joint_name = joint.get("name")
            if joint_name:
                joints[joint_name] = float(joint.get("value") or "0")
        if group and name:
            states[(group, name)] = joints
    return states


def normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-8:
        raise ValueError("姿态四元数长度为零")
    return qx / norm, qy / norm, qz / norm, qw / norm

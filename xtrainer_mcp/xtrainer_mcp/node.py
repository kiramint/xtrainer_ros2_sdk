"""MCP 用的 ROS 节点。只做客户端，不起驱动，也不建 MoveItPy。"""

from __future__ import annotations

import logging
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

from xtrainer_control.robot_control import RobotController
from xtrainer_gripper.gripper_control import GripperController

from xtrainer_mcp.cameras import CameraHub
from xtrainer_mcp.geometry import ARM_JOINTS, ARMS, f6
from xtrainer_mcp.motion import MotionClient
from xtrainer_mcp.servo import ServoStreamer

_MODE_NAMES = {
    -1: "UNKNOWN",
    0: "NO_DATA",
    1: "INIT",
    2: "BRAKE_OPEN",
    3: "POWEROFF",
    4: "DISABLED",
    5: "IDLE",
    6: "DRAG",
    7: "RUNNING",
    8: "SINGLE_MOVE",
    9: "ERROR",
    10: "PAUSE",
    11: "COLLISION",
}
_GRIPPER_SIDES = ("left", "right", "both")


class MotionBusy(RuntimeError):
    pass


class MotionGate:
    def __init__(self):
        self._lock = threading.Lock()

    @property
    def busy(self) -> bool:
        return self._lock.locked()

    def __enter__(self):
        if not self._lock.acquire(blocking=False):
            raise MotionBusy("机械臂正在执行其他动作")
        return self

    def __exit__(self, exc_type, exc, tb):
        self._lock.release()
        return False


def load_description() -> tuple[str, str]:
    """读 xtrainer_moveit 的 URDF/SRDF。只取模型，不创建 MoveItPy。"""
    logging.basicConfig(level=logging.INFO, stream=sys.stderr, force=True)
    from moveit_configs_utils import MoveItConfigsBuilder

    saved = sys.stdout
    sys.stdout = sys.stderr
    try:
        configs = MoveItConfigsBuilder(
            "x_trainer", package_name="xtrainer_moveit"
        ).to_moveit_configs()
    finally:
        sys.stdout = saved
    urdf = configs.robot_description.get("robot_description")
    srdf = configs.robot_description_semantic.get("robot_description_semantic")
    if not isinstance(urdf, str) or not isinstance(srdf, str):
        raise RuntimeError("MoveIt 配置里没有字符串形式的 URDF/SRDF")
    return urdf, srdf


def _arms(arm: str) -> list[str]:
    if arm == "both":
        return list(ARMS)
    if arm not in ARMS:
        raise ValueError(f"arm 只能是 Arm1、Arm2 或 both，收到 {arm!r}")
    return [arm]


def _sides(side: str) -> tuple[str, ...]:
    if side not in _GRIPPER_SIDES:
        raise ValueError(f"side 只能是 left、right 或 both，收到 {side!r}")
    if side == "both":
        return ("left", "right")
    return (side,)


class XTrainerMcpNode(Node):
    def __init__(self):
        super().__init__("xtrainer_mcp")
        urdf, srdf = load_description()
        self._gate = MotionGate()
        self._joint_lock = threading.Lock()
        self._joints: dict[str, float] = {}
        # rclpy.Node already uses _clients for its service-client list.
        self._callback_group = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.cameras = CameraHub(self)
        self.motion = MotionClient(self, urdf, srdf, self._callback_group)
        self.servo = ServoStreamer(self, self._callback_group)
        self.robot = RobotController(self)
        self.gripper = GripperController(self)
        self.get_logger().info("xtrainer_mcp 已连接 ROS 图，等待 agent 调用")

    def wait_future(self, future, timeout: float) -> bool:
        """节点已经挂在 executor 上时只轮询 future，避免再 spin 一次。"""
        if self.executor is not None:
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() >= deadline or not rclpy.ok():
                    return False
                time.sleep(0.005)
            return True
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done()

    def joints(self) -> dict[str, float]:
        with self._joint_lock:
            return dict(self._joints)

    def _on_joints(self, msg: JointState) -> None:
        with self._joint_lock:
            for name, position in zip(msg.name, msg.position):
                self._joints[name] = float(position)

    def _guarded(self, fn):
        try:
            with self._gate:
                return fn()
        except MotionBusy as exc:
            return {"ok": False, "error": str(exc)}

    def get_camera_frame(self, camera: str) -> dict:
        return self.cameras.get_frame(camera)

    def project_pixel(self, camera: str, u: int, v: int, window: int = 5) -> dict:
        return self.cameras.project_pixel(camera, u, v, window)

    def transform_optical_point(self, camera: str, x: float, y: float, z: float) -> dict:
        return self.cameras.transform_optical_point(camera, x, y, z)

    def get_tcp_pose(self, arm: str, link: str = "tcp") -> dict:
        return self.motion.tcp_pose(arm, link, self.joints())

    def fk(self, arm: str, positions: list[float], link: str = "tcp") -> dict:
        return self.motion.fk(arm, positions, link, self.joints())

    def ik(
        self,
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
        return self.motion.ik(arm, (x, y, z), (qx, qy, qz, qw), link, self.joints())

    def get_status(self) -> dict:
        joints = self.joints()
        arms = {}
        for arm in ARMS:
            mode = self.robot.get_robot_mode(arm, timeout=1.0)
            tcp = self.motion.tcp_pose(arm, "tcp", joints)
            arms[arm] = {
                "mode": mode,
                "mode_name": _MODE_NAMES.get(mode, str(mode)),
                "joints": {
                    name: f6(joints[name])
                    for name in ARM_JOINTS[arm]
                    if name in joints
                },
                "tcp": tcp.get("pose") if tcp.get("ok") else {"ok": False, "error": tcp.get("error", "")},
            }
        return {
            "ok": True,
            "motion_busy": self._gate.busy,
            "arms": arms,
            "gripper": self._gripper_snapshot(("left", "right")),
            "plans": self.motion.plan_ids(),
        }

    def plan_joints(self, arm: str, positions: list[float], planner: str = "ompl") -> dict:
        return self.motion.plan_joints(arm, positions, planner, self.joints())

    def plan_pose(
        self,
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
        return self.motion.plan_pose(
            arm, (x, y, z), (qx, qy, qz, qw), link, planner, self.joints()
        )

    def plan_named(self, arm: str, name: str, planner: str = "ompl") -> dict:
        return self.motion.plan_named(arm, name, planner, self.joints())

    def execute_plan(self, plan_id: str) -> dict:
        return self._guarded(lambda: self.motion.execute(plan_id, self.joints()))

    def check_self_collision(self) -> dict:
        return self.motion.check_current(self.joints())

    def check_joints(self, arm: str, positions: list[float]) -> dict:
        return self.motion.check_arm_positions(arm, positions, self.joints())

    def check_plan(self, plan_id: str) -> dict:
        return self.motion.check_plan(plan_id, self.joints())

    def servo_joints(self, arm: str, points: list[list[float]], dt: float) -> dict:
        current_map = self.joints()
        names = ARM_JOINTS[arm] if arm in ARM_JOINTS else None
        if names is None:
            raise ValueError(f"arm 只能是 Arm1 或 Arm2，收到 {arm!r}")
        missing = [name for name in names if name not in current_map]
        if missing:
            return {"ok": False, "error": f"/joint_states 还没有 {missing}"}
        current = [current_map[name] for name in names]

        def collision_at(point):
            return self.motion.check_arm_positions(arm, list(point), self.joints())

        return self._guarded(
            lambda: self.servo.stream(arm, points, dt, current, collision_at)
        )

    def gripper_open(self, side: str, speed: float = 1.0) -> dict:
        speed = _speed(speed)
        _sides(side)

        def send():
            if side == "both":
                self.gripper.open_both(speed)
            else:
                self.gripper.open(side, speed)
            return {"ok": True, "action": "open", "side": side, "speed": speed}

        return self._guarded(send)

    def gripper_close(self, side: str, speed: float = 1.0) -> dict:
        speed = _speed(speed)

        def send():
            if side == "both":
                self.gripper.close_both(speed)
            else:
                self.gripper.close(side, speed)
            return {"ok": True, "action": "close", "side": side, "speed": speed}

        _sides(side)
        return self._guarded(send)

    def gripper_set_torque(self, side: str, limit: int) -> dict:
        limit = int(limit)
        if limit < 0 or limit > 1000:
            raise ValueError("力矩限制必须在 0 到 1000")

        def send():
            if side == "both":
                self.gripper.set_torque_both(limit)
            else:
                self.gripper.set_torque(side, limit)
            return {"ok": True, "side": side, "torque": limit}

        _sides(side)
        return self._guarded(send)

    def gripper_get_state(self, side: str = "both") -> dict:
        return {"ok": True, "gripper": self._gripper_snapshot(_sides(side))}

    def clear_error(self, arm: str = "both") -> dict:
        names = _arms(arm)

        def send():
            results = {}
            for name in names:
                results[name] = {"ok": bool(self.robot.clear_error_arm(name))}
            return {"ok": all(item["ok"] for item in results.values()), "arms": results}

        return self._guarded(send)

    def unlock_brakes(self, arm: str = "both") -> dict:
        names = _arms(arm)

        def send():
            results = {}
            for name in names:
                ok = bool(self.robot.enable_arm(name))
                mode = self.robot.get_robot_mode(name, timeout=3.0)
                results[name] = {
                    "ok": ok,
                    "mode": mode,
                    "mode_name": _MODE_NAMES.get(mode, str(mode)),
                }
            return {"ok": all(item["ok"] for item in results.values()), "arms": results}

        return self._guarded(send)

    def lock_brakes(self, arm: str = "both") -> dict:
        names = _arms(arm)

        def send():
            results = {}
            for name in names:
                ok = bool(self.robot.disable_arm(name))
                mode = self.robot.get_robot_mode(name, timeout=3.0)
                results[name] = {
                    "ok": ok,
                    "mode": mode,
                    "mode_name": _MODE_NAMES.get(mode, str(mode)),
                }
            return {"ok": all(item["ok"] for item in results.values()), "arms": results}

        return self._guarded(send)

    def get_robot_mode(self, arm: str = "both") -> dict:
        results = {}
        for name in _arms(arm):
            mode = self.robot.get_robot_mode(name, timeout=2.0)
            results[name] = {"mode": mode, "mode_name": _MODE_NAMES.get(mode, str(mode))}
        return {"ok": True, "arms": results}

    def _gripper_snapshot(self, sides: tuple[str, ...]) -> dict:
        out = {}
        for side in sides:
            out[side] = {
                "position": f6(self.gripper.read_position(side)),
                "load": f6(self.gripper.read_load(side)),
                "raw_position": int(self.gripper.read_raw_position(side)),
                "raw_load": int(self.gripper.read_raw_load(side)),
                "status": self.gripper.read_status(side),
            }
        return out


def _speed(speed: float) -> float:
    speed = float(speed)
    if speed < 0.0 or speed > 1.0:
        raise ValueError("speed 必须在 0 到 1")
    return speed

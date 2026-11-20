"""短程 ServoJ。关节角在这里从弧度换成度，只在这一处换。"""

from __future__ import annotations

import math
import time
from typing import Callable, Sequence

import rclpy
from dobot_msgs_v4.srv import ServoJ
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from xtrainer_mcp.geometry import f6, require_arm, validate_servo_path

_SERVICE = {
    "Arm1": "/Arm1/dobot_bringup_ros2/srv/ServoJ",
    "Arm2": "/Arm2/dobot_bringup_ros2/srv/ServoJ",
}


class ServoStreamer:
    def __init__(self, node: Node, callback_group: CallbackGroup):
        self._node = node
        self._clients = {
            arm: node.create_client(ServoJ, service, callback_group=callback_group)
            for arm, service in _SERVICE.items()
        }

    def stream(
        self,
        arm: str,
        points: Sequence[Sequence[float]],
        dt: float,
        current: Sequence[float],
        collision_at: Callable[[Sequence[float]], dict],
    ) -> dict:
        arm = require_arm(arm)
        path = [list(map(float, point)) for point in points]
        error = validate_servo_path(current, path, float(dt))
        if error:
            return {"ok": False, "error": error, "arm": arm}
        for index, point in enumerate(path):
            collision = collision_at(point)
            if not collision.get("ok", False):
                collision["error"] = f"第 {index} 个点无法做自碰撞检查: {collision.get('error', '')}"
                collision["ok"] = False
                return collision
            if not collision.get("valid", False):
                return {
                    "ok": False,
                    "error": f"第 {index} 个点自碰撞，整段 ServoJ 都没有发送",
                    "arm": arm,
                    "index": index,
                    "self_collision": collision,
                }
        client = self._clients[arm]
        if not client.wait_for_service(timeout_sec=2.0):
            return {"ok": False, "error": f"{_SERVICE[arm]} 不可用", "arm": arm}
        start = time.monotonic()
        for index, point in enumerate(path):
            wait = start + index * float(dt) - time.monotonic()
            if wait > 0.0:
                time.sleep(wait)
            if not rclpy.ok():
                return {"ok": False, "error": "ROS 已关闭", "arm": arm, "sent": index}
            request = ServoJ.Request()
            degrees = [math.degrees(value) for value in point]
            request.a, request.b, request.c, request.d, request.e, request.f = degrees
            request.param_value = [f"t={float(dt):.4f}"]
            client.call_async(request)
        return {
            "ok": True,
            "arm": arm,
            "sent": len(path),
            "dt": f6(dt),
            "duration_sec": f6(len(path) * float(dt)),
        }

"""move_group 的规划、执行、FK、IK 和整机自碰撞。

不在本进程里建 MoveItPy，避免再出现一个 TrajectoryExecutionManager。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import (
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory

from xtrainer_mcp.geometry import (
    ARM_JOINTS,
    BASE_FRAME,
    EXECUTE_START_TOLERANCE,
    PLANNERS,
    PLAN_JSON_MAX_POINTS,
    UrdfKinematics,
    f6,
    normalize_quaternion,
    parse_named_states,
    require_arm,
    resolve_link,
    select_indices,
    trajectory_json,
)

_PLAN_CACHE = 8
_VELOCITY_SCALE = 0.3
_ACCELERATION_SCALE = 0.1
_POSITION_TOLERANCE = 0.001
_ORIENTATION_TOLERANCE = 0.01
_COLLISION_SAMPLES = 40


def error_name(code: int) -> str:
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == code:
            return name
    return str(code)


@dataclass
class _Plan:
    arm: str
    planner: str
    pipeline_id: str
    planner_id: str
    trajectory: JointTrajectory
    start_state: RobotState
    planning_time: float
    end_link: str
    payload: dict = field(default_factory=dict)


class MotionClient:
    def __init__(
        self,
        node: Node,
        urdf_xml: str,
        srdf_xml: str,
        callback_group: CallbackGroup,
    ):
        self._node = node
        self._kinematics = UrdfKinematics(urdf_xml)
        self._named = parse_named_states(srdf_xml)
        self._cb = callback_group
        self._plan_lock = threading.Lock()
        self._plans: dict[str, _Plan] = {}
        self._plan_seq = 0
        self._fk_down_until = 0.0

        self._plan_client = node.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=callback_group
        )
        self._fk_client = node.create_client(
            GetPositionFK, "/compute_fk", callback_group=callback_group
        )
        self._ik_client = node.create_client(
            GetPositionIK, "/compute_ik", callback_group=callback_group
        )
        self._validity_client = node.create_client(
            GetStateValidity, "/check_state_validity", callback_group=callback_group
        )
        self._execute_client = ActionClient(
            node,
            ExecuteTrajectory,
            "/execute_trajectory",
            callback_group=callback_group,
        )
        self._display_pub = node.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )

    def named_target(self, arm: str, name: str) -> dict[str, float]:
        arm = require_arm(arm)
        state = self._named.get((arm, name))
        if state is None:
            known = sorted(key for group, key in self._named if group == arm)
            raise ValueError(f"{arm} 没有名为 {name!r} 的姿态，已有: {known}")
        return dict(state)

    def local_fk(self, link: str, joints: dict[str, float]) -> dict:
        return self._kinematics.link_pose(link, joints)

    def fk(self, arm: str, positions: list[float], link: Optional[str], joints: dict) -> dict:
        arm = require_arm(arm)
        link_name = resolve_link(arm, link)
        merged = dict(joints)
        names = ARM_JOINTS[arm]
        if len(positions) != len(names):
            raise ValueError(f"{arm} 需要 {len(names)} 个关节角，收到 {len(positions)} 个")
        for name, value in zip(names, positions):
            merged[name] = float(value)
        pose = self._fk_service(link_name, merged)
        source = "compute_fk"
        if pose is None:
            pose = self.local_fk(link_name, merged)
            source = "urdf"
        return {"ok": True, "arm": arm, "link": link_name, "source": source, "pose": pose, "joints": {n: f6(merged[n]) for n in names}}

    def tcp_pose(self, arm: str, link: Optional[str], joints: dict) -> dict:
        names = ARM_JOINTS[require_arm(arm)]
        missing = [name for name in names if name not in joints]
        if missing:
            return {"ok": False, "error": f"/joint_states 还没有 {missing}"}
        return self.fk(arm, [joints[name] for name in names], link, joints)

    def ik(
        self,
        arm: str,
        position: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
        link: Optional[str],
        joints: dict,
    ) -> dict:
        arm = require_arm(arm)
        link_name = resolve_link(arm, link)
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = arm
        ik.ik_link_name = link_name
        ik.avoid_collisions = True
        ik.robot_state = self._robot_state(joints)
        ik.pose_stamped = self._pose(position, quaternion)
        ik.timeout.sec = 1
        response = self._call(self._ik_client, request, timeout=3.0)
        if response is None:
            return {"ok": False, "error": "/compute_ik 不可用或超时"}
        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return {"ok": False, "error": f"IK 失败: {error_name(code)}", "error_code": error_name(code)}
        solved = {
            name: float(value)
            for name, value in zip(response.solution.joint_state.name, response.solution.joint_state.position)
        }
        names = ARM_JOINTS[arm]
        missing = [name for name in names if name not in solved]
        if missing:
            return {"ok": False, "error": f"IK 解里没有 {missing}"}
        merged = dict(joints)
        merged.update(solved)
        collision = self.check_joints_map(merged)
        return {
            "ok": True,
            "arm": arm,
            "link": link_name,
            "joints": {name: f6(solved[name]) for name in names},
            "self_collision": collision,
        }

    def check_current(self, joints: dict) -> dict:
        return self.check_joints_map(joints)

    def check_arm_positions(self, arm: str, positions: list[float], joints: dict) -> dict:
        arm = require_arm(arm)
        names = ARM_JOINTS[arm]
        if len(positions) != len(names):
            raise ValueError(f"{arm} 需要 {len(names)} 个关节角，收到 {len(positions)} 个")
        merged = dict(joints)
        for name, value in zip(names, positions):
            merged[name] = float(value)
        result = self.check_joints_map(merged)
        result["arm"] = arm
        return result

    def check_joints_map(self, joints: dict) -> dict:
        request = GetStateValidity.Request()
        request.robot_state = self._robot_state(joints)
        request.group_name = ""
        response = self._call(self._validity_client, request, timeout=3.0)
        if response is None:
            return {"ok": False, "error": "/check_state_validity 不可用或超时"}
        return {
            "ok": True,
            "valid": bool(response.valid),
            "contacts": [_contact(item) for item in response.contacts],
        }

    def check_plan(self, plan_id: str, joints: dict) -> dict:
        plan = self._get_plan(plan_id)
        if plan is None:
            return {"ok": False, "error": f"没有计划 {plan_id}"}
        samples = self._sample_maps(plan.trajectory, joints)
        contacts = []
        for sample in samples:
            result = self.check_joints_map(sample)
            if not result.get("ok", False):
                return result
            if not result["valid"]:
                contacts.extend(result["contacts"])
        return {
            "ok": True,
            "plan_id": plan_id,
            "valid": not contacts,
            "samples": len(samples),
            "contacts": contacts,
        }

    def plan_joints(self, arm: str, positions: list[float], planner: str, joints: dict) -> dict:
        arm = require_arm(arm)
        names = ARM_JOINTS[arm]
        if len(positions) != len(names):
            raise ValueError(f"{arm} 需要 {len(names)} 个关节角，收到 {len(positions)} 个")
        goal = self._joint_goal(names, positions)
        return self._plan(arm, goal, planner, joints, resolve_link(arm, "tcp"))

    def plan_named(self, arm: str, name: str, planner: str, joints: dict) -> dict:
        target = self.named_target(arm, name)
        names = ARM_JOINTS[arm]
        missing = [joint for joint in names if joint not in target]
        if missing:
            return {"ok": False, "error": f"{name} 缺少关节 {missing}"}
        return self.plan_joints(arm, [target[joint] for joint in names], planner, joints)

    def plan_pose(
        self,
        arm: str,
        position: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
        link: Optional[str],
        planner: str,
        joints: dict,
    ) -> dict:
        arm = require_arm(arm)
        link_name = resolve_link(arm, link)
        goal = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_FRAME
        position_constraint.link_name = link_name
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [_POSITION_TOLERANCE]
        position_constraint.constraint_region.primitives = [sphere]
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0
        position_constraint.constraint_region.primitive_poses = [pose.pose]
        position_constraint.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = BASE_FRAME
        orientation.link_name = link_name
        qx, qy, qz, qw = normalize_quaternion(*quaternion)
        orientation.orientation.x = qx
        orientation.orientation.y = qy
        orientation.orientation.z = qz
        orientation.orientation.w = qw
        orientation.absolute_x_axis_tolerance = _ORIENTATION_TOLERANCE
        orientation.absolute_y_axis_tolerance = _ORIENTATION_TOLERANCE
        orientation.absolute_z_axis_tolerance = _ORIENTATION_TOLERANCE
        orientation.weight = 1.0
        goal.position_constraints = [position_constraint]
        goal.orientation_constraints = [orientation]
        return self._plan(arm, goal, planner, joints, link_name)

    def execute(self, plan_id: str, joints: dict) -> dict:
        plan = self._get_plan(plan_id)
        if plan is None:
            return {"ok": False, "error": f"没有计划 {plan_id}"}
        drift = self._start_drift(plan.trajectory, joints)
        if drift is not None:
            return {"ok": False, "error": drift, "plan_id": plan_id}
        collision = self.check_plan(plan_id, joints)
        if not collision.get("ok", False):
            collision["plan_id"] = plan_id
            return collision
        if not collision.get("valid", False):
            return {
                "ok": False,
                "error": "轨迹自碰撞，拒绝执行",
                "plan_id": plan_id,
                "self_collision": collision,
            }
        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            return {"ok": False, "error": "/execute_trajectory 不可用", "plan_id": plan_id}
        goal = ExecuteTrajectory.Goal()
        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = plan.trajectory
        goal.trajectory = robot_trajectory
        send_future = self._execute_client.send_goal_async(goal)
        if not self._node.wait_future(send_future, 10.0):
            return {"ok": False, "error": "发送轨迹超时", "plan_id": plan_id}
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {"ok": False, "error": "move_group 拒绝了这条轨迹", "plan_id": plan_id}
        duration = 0.0
        if plan.trajectory.points:
            last = plan.trajectory.points[-1].time_from_start
            duration = float(last.sec) + float(last.nanosec) * 1e-9
        result_future = handle.get_result_async()
        if not self._node.wait_future(result_future, max(30.0, duration + 30.0)):
            return {"ok": False, "error": "轨迹执行超时", "plan_id": plan_id}
        wrapped = result_future.result()
        code = int(wrapped.result.error_code.val)
        status = int(wrapped.status)
        ok = code == MoveItErrorCodes.SUCCESS and status == GoalStatus.STATUS_SUCCEEDED
        return {
            "ok": ok,
            "plan_id": plan_id,
            "error_code": error_name(code),
            "status": status,
            "error": "" if ok else f"执行失败: {error_name(code)}",
        }

    def plan_ids(self) -> list[str]:
        with self._plan_lock:
            return list(self._plans)

    def _plan(self, arm: str, goal: Constraints, planner: str, joints: dict, link: str) -> dict:
        if planner not in PLANNERS:
            raise ValueError(f"planner 只能是 {', '.join(PLANNERS)}，收到 {planner!r}")
        pipeline_id, planner_id = PLANNERS[planner]
        motion = MotionPlanRequest()
        motion.group_name = arm
        motion.pipeline_id = pipeline_id
        motion.planner_id = planner_id
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = _VELOCITY_SCALE
        motion.max_acceleration_scaling_factor = _ACCELERATION_SCALE
        motion.start_state = self._robot_state(joints)
        workspace = motion.workspace_parameters
        workspace.header.frame_id = BASE_FRAME
        workspace.min_corner.x = -2.0
        workspace.min_corner.y = -2.0
        workspace.min_corner.z = -2.0
        workspace.max_corner.x = 2.0
        workspace.max_corner.y = 2.0
        workspace.max_corner.z = 2.0
        motion.workspace_parameters = workspace
        motion.goal_constraints = [goal]
        missing = [name for name in ARM_JOINTS[arm] if name not in joints]
        if missing:
            return {"ok": False, "error": f"/joint_states 还没有 {missing}"}
        request = GetMotionPlan.Request()
        request.motion_plan_request = motion
        response = self._call(self._plan_client, request, timeout=15.0)
        if response is None:
            return {"ok": False, "error": "/plan_kinematic_path 不可用或超时"}
        result = response.motion_plan_response
        code = int(result.error_code.val)
        trajectory = result.trajectory.joint_trajectory
        if code != MoveItErrorCodes.SUCCESS or not trajectory.points:
            return {
                "ok": False,
                "error": f"规划失败: {error_name(code)}",
                "error_code": error_name(code),
                "arm": arm,
                "planner": planner,
            }
        self._publish_display(result.trajectory_start, result.trajectory)
        record = _Plan(
            arm=arm,
            planner=planner,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            trajectory=trajectory,
            start_state=result.trajectory_start,
            planning_time=float(result.planning_time),
            end_link=link,
        )
        plan_id = self._store(record)
        payload = self._describe(plan_id, record, joints)
        record.payload = payload
        return payload

    def _describe(self, plan_id: str, plan: _Plan, joints: dict) -> dict:
        times = []
        positions = []
        velocities = []
        for point in plan.trajectory.points:
            times.append(float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9)
            positions.append(list(point.positions))
            velocities.append(list(point.velocities))
        body = trajectory_json(
            plan.trajectory.joint_names,
            times,
            positions,
            velocities,
            PLAN_JSON_MAX_POINTS,
        )
        merged = dict(joints)
        if positions:
            for name, value in zip(plan.trajectory.joint_names, positions[-1]):
                merged[name] = float(value)
        end_pose = self._fk_service(plan.end_link, merged) or self.local_fk(plan.end_link, merged)
        goal_collision = self.check_joints_map(merged)
        body.update(
            {
                "ok": True,
                "plan_id": plan_id,
                "arm": plan.arm,
                "planner": plan.planner,
                "pipeline_id": plan.pipeline_id,
                "planner_id": plan.planner_id,
                "planning_time_sec": f6(plan.planning_time),
                "end_link": plan.end_link,
                "end_pose": end_pose,
                "self_collision": goal_collision,
            }
        )
        return body

    def _publish_display(self, start_state: RobotState, trajectory) -> None:
        message = DisplayTrajectory()
        message.model_id = "x_trainer"
        message.trajectory_start = start_state
        message.trajectory = [trajectory]
        self._display_pub.publish(message)

    def _store(self, plan: _Plan) -> str:
        with self._plan_lock:
            self._plan_seq += 1
            plan_id = f"p{self._plan_seq}"
            self._plans[plan_id] = plan
            while len(self._plans) > _PLAN_CACHE:
                del self._plans[next(iter(self._plans))]
            return plan_id

    def _get_plan(self, plan_id: str) -> Optional[_Plan]:
        with self._plan_lock:
            return self._plans.get(plan_id)

    def _start_drift(self, trajectory: JointTrajectory, joints: dict) -> Optional[str]:
        if not trajectory.points:
            return "轨迹没有点"
        first = trajectory.points[0]
        drifted = []
        for name, value in zip(trajectory.joint_names, first.positions):
            if name not in joints:
                return f"/joint_states 没有 {name}，不能确认起点"
            delta = abs(float(joints[name]) - float(value))
            if delta > EXECUTE_START_TOLERANCE:
                drifted.append(f"{name} 差 {delta:.4f} rad")
        if drifted:
            return "当前关节和轨迹起点相差超过 0.05 rad: " + ", ".join(drifted)
        return None

    def _sample_maps(self, trajectory: JointTrajectory, joints: dict) -> list[dict]:
        count = len(trajectory.points)
        indices = select_indices(count, min(count, _COLLISION_SAMPLES)) if count else []
        samples = []
        for idx in indices:
            merged = dict(joints)
            point = trajectory.points[idx]
            for name, value in zip(trajectory.joint_names, point.positions):
                merged[name] = float(value)
            samples.append(merged)
        return samples

    def _fk_service(self, link: str, joints: dict) -> Optional[dict]:
        if time.monotonic() < self._fk_down_until:
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = BASE_FRAME
        request.fk_link_names = [link]
        request.robot_state = self._robot_state(joints)
        response = self._call(self._fk_client, request, timeout=0.5)
        if response is None or int(response.error_code.val) != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            self._fk_down_until = time.monotonic() + 2.0
            return None
        self._fk_down_until = 0.0
        pose = response.pose_stamped[0].pose
        return {
            "frame": response.pose_stamped[0].header.frame_id or BASE_FRAME,
            "position": {"x": f6(pose.position.x), "y": f6(pose.position.y), "z": f6(pose.position.z)},
            "orientation": {
                "x": f6(pose.orientation.x),
                "y": f6(pose.orientation.y),
                "z": f6(pose.orientation.z),
                "w": f6(pose.orientation.w),
            },
        }

    def _robot_state(self, joints: dict) -> RobotState:
        state = RobotState()
        state.is_diff = False
        state.joint_state = JointState()
        names = list(joints)
        state.joint_state.name = names
        state.joint_state.position = [float(joints[name]) for name in names]
        return state

    def _joint_goal(self, names: list[str], positions: list[float]) -> Constraints:
        goal = Constraints()
        for name, position in zip(names, positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 1e-4
            constraint.tolerance_below = 1e-4
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        return goal

    def _pose(self, position, quaternion) -> PoseStamped:
        qx, qy, qz, qw = normalize_quaternion(*quaternion)
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _call(self, client, request, timeout: float):
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline or not rclpy.ok():
                return None
            time.sleep(0.05)
        future = client.call_async(request)
        remaining = max(0.1, deadline - time.monotonic())
        if not self._node.wait_future(future, remaining):
            return None
        return future.result()


def _contact(item) -> dict:
    return {
        "link_1": item.contact_body_1,
        "link_2": item.contact_body_2,
        "depth": f6(item.depth),
        "position": [f6(item.position.x), f6(item.position.y), f6(item.position.z)],
    }

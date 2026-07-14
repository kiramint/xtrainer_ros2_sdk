#!/usr/bin/env python3
"""
xtrainer_bridge_node: 双臂桥接节点

功能:
  1. 订阅 Arm1/Arm2 各自命名空间下的 joint_states_robot，
     合并为统一的 /joint_states 话题 (MoveIt 所需)
  2. 为 Arm1/Arm2 各创建一个 FollowJointTrajectory Action Server，
     将 MoveIt 规划的轨迹通过 ServoJ 服务发送至对应机械臂

架构:
  Arm1/dobot_bringup_ros2 ──► joint_states_robot ──┐
                                                    ├──► /joint_states (合并)
  Arm2/dobot_bringup_ros2 ──► joint_states_robot ──┘

  MoveIt ──► /Arm1_controller/follow_joint_trajectory (action) ──► /Arm1/.../ServoJ
  MoveIt ──► /Arm2_controller/follow_joint_trajectory (action) ──► /Arm2/.../ServoJ
"""

import threading
import time
from collections import OrderedDict

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from dobot_msgs_v4.srv import ServoJ


class XTrainerBridge(Node):
    """双臂桥接节点"""

    def __init__(self):
        super().__init__('xtrainer_bridge')

        # ── 参数声明 ──
        self.declare_parameter('arm1_joint_names', ['J1_1', 'J1_2', 'J1_3', 'J1_4', 'J1_5', 'J1_6'])
        self.declare_parameter('arm2_joint_names', ['J2_1', 'J2_2', 'J2_3', 'J2_4', 'J2_5', 'J2_6'])
        self.declare_parameter('arm1_servoj_service', '/Arm1/dobot_bringup_ros2/srv/ServoJ')
        self.declare_parameter('arm2_servoj_service', '/Arm2/dobot_bringup_ros2/srv/ServoJ')
        self.declare_parameter('arm1_joint_state_topic', '/Arm1/joint_states_robot')
        self.declare_parameter('arm2_joint_state_topic', '/Arm2/joint_states_robot')
        self.declare_parameter('control_frequency', 50.0)       # 控制循环频率 (Hz)
        self.declare_parameter('action_timeout', 30.0)          # action 超时 (秒)

        self._arm1_joint_names = self.get_parameter('arm1_joint_names').value
        self._arm2_joint_names = self.get_parameter('arm2_joint_names').value
        self._arm1_servo_srv = self.get_parameter('arm1_servoj_service').value
        self._arm2_servo_srv = self.get_parameter('arm2_servoj_service').value
        self._arm1_state_topic = self.get_parameter('arm1_joint_state_topic').value
        self._arm2_state_topic = self.get_parameter('arm2_joint_state_topic').value
        self._ctrl_freq = self.get_parameter('control_frequency').value
        self._action_timeout = self.get_parameter('action_timeout').value

        self._all_joint_names = self._arm1_joint_names + self._arm2_joint_names

        # ── 关节状态缓存 ──
        self._arm1_positions = OrderedDict((n, 0.0) for n in self._arm1_joint_names)
        self._arm2_positions = OrderedDict((n, 0.0) for n in self._arm2_joint_names)
        self._arm1_lock = threading.Lock()
        self._arm2_lock = threading.Lock()

        # ── 发布者 ──
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── 订阅者 ──
        self._arm1_sub = self.create_subscription(
            JointState, self._arm1_state_topic,
            self._arm1_callback, 10)
        self._arm2_sub = self.create_subscription(
            JointState, self._arm2_state_topic,
            self._arm2_callback, 10)

        # ── ServoJ 服务客户端 ──
        self._arm1_servo_client = self.create_client(ServoJ, self._arm1_servo_srv)
        self._arm2_servo_client = self.create_client(ServoJ, self._arm2_servo_srv)

        # ── FollowJointTrajectory Action Server ──
        self._cb_group = ReentrantCallbackGroup()
        self._arm1_action_server = ActionServer(
            self, FollowJointTrajectory, '/Arm1_controller/follow_joint_trajectory',
            execute_callback=self._arm1_execute_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group)
        self._arm2_action_server = ActionServer(
            self, FollowJointTrajectory, '/Arm2_controller/follow_joint_trajectory',
            execute_callback=self._arm2_execute_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group)

        # ── 发布定时器 ──
        self._pub_timer = self.create_timer(1.0 / self._ctrl_freq, self._publish_joint_states)

        self.get_logger().info('XTrainerBridge 初始化完成')
        self.get_logger().info(f'  左臂关节: {self._arm1_joint_names}')
        self.get_logger().info(f'  右臂关节: {self._arm2_joint_names}')
        self.get_logger().info(f'  Action: /Arm1_controller/follow_joint_trajectory')
        self.get_logger().info(f'  Action: /Arm2_controller/follow_joint_trajectory')

    # ═══════════════════════════════════════════════════════════════
    #  关节状态回调
    # ═══════════════════════════════════════════════════════════════

    def _arm1_callback(self, msg: JointState):
        with self._arm1_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._arm1_positions:
                    self._arm1_positions[name] = pos

    def _arm2_callback(self, msg: JointState):
        with self._arm2_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._arm2_positions:
                    self._arm2_positions[name] = pos

    def _publish_joint_states(self):
        """定时发布合并后的 /joint_states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        with self._arm1_lock:
            arm1_pos = list(self._arm1_positions.values())
        with self._arm2_lock:
            arm2_pos = list(self._arm2_positions.values())

        msg.name = self._all_joint_names
        msg.position = arm1_pos + arm2_pos
        msg.velocity = [0.0] * len(self._all_joint_names)
        msg.effort = [0.0] * len(self._all_joint_names)

        self._joint_state_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    #  Action 回调
    # ═══════════════════════════════════════════════════════════════

    def _goal_callback(self, goal_request):
        self.get_logger().info('收到 FollowJointTrajectory goal')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    # ── 轨迹插补与执行 ──

    def _interpolate_trajectory(self, points: list[JointTrajectoryPoint],
                                 joint_names: list[str],
                                 servo_client,
                                 feedback_cb,
                                 is_canceled_cb) -> bool:
        """
        在轨迹点之间进行线性插补，通过 ServoJ 逐点发送。

        Returns:
            True  — 执行成功完成
            False — 被取消或失败
        """
        if not points:
            return True

        # 等待 ServoJ 服务
        if not servo_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('ServoJ 服务不可用')
            return False

        prev_point = None
        for idx, point in enumerate(points):
            if is_canceled_cb():
                self.get_logger().info('轨迹执行被取消')
                return False

            # 计算本段插补步数
            if prev_point is None:
                # 第一个点：直接到位
                steps = 1
            else:
                dt = self._time_from_sec_nsec(
                    point.time_from_start.sec,
                    point.time_from_start.nanosec)
                prev_dt = self._time_from_sec_nsec(
                    prev_point.time_from_start.sec,
                    prev_point.time_from_start.nanosec)
                duration = dt - prev_dt
                if duration <= 0.0:
                    steps = 1
                else:
                    steps = max(1, int(duration * self._ctrl_freq))

            # 插补
            if steps <= 1:
                self._send_servoj(servo_client, point.positions,
                                  joint_names)
            else:
                start_pos = list(prev_point.positions) if prev_point else list(point.positions)
                end_pos = list(point.positions)
                for step in range(1, steps + 1):
                    if is_canceled_cb():
                        return False
                    alpha = step / steps
                    interp = [s + (e - s) * alpha
                              for s, e in zip(start_pos, end_pos)]
                    self._send_servoj(servo_client, interp, joint_names)
                    time.sleep(1.0 / self._ctrl_freq)

            prev_point = point

            # 反馈
            if feedback_cb:
                feedback_cb(idx, point)

        return True

    def _send_servoj(self, client, positions: list[float], joint_names: list[str]):
        """发送 ServoJ 请求"""
        req = ServoJ.Request()
        req.a = positions[0] if len(positions) > 0 else 0.0
        req.b = positions[1] if len(positions) > 1 else 0.0
        req.c = positions[2] if len(positions) > 2 else 0.0
        req.d = positions[3] if len(positions) > 3 else 0.0
        req.e = positions[4] if len(positions) > 4 else 0.0
        req.f = positions[5] if len(positions) > 5 else 0.0
        req.param_value = joint_names if joint_names else []
        client.call_async(req)  # fire-and-forget 实现高频控制

    @staticmethod
    def _time_from_sec_nsec(sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) * 1e-9

    # ── Execute Callbacks ──

    def _arm1_execute_cb(self, goal_handle):
        return self._execute_trajectory(
            goal_handle, self._arm1_joint_names, self._arm1_servo_client)

    def _arm2_execute_cb(self, goal_handle):
        return self._execute_trajectory(
            goal_handle, self._arm2_joint_names, self._arm2_servo_client)

    def _execute_trajectory(self, goal_handle, joint_names, servo_client):
        """通用轨迹执行"""
        from control_msgs.action import FollowJointTrajectory

        request = goal_handle.request
        trajectory = request.trajectory
        points = trajectory.points

        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        self.get_logger().info(
            f'执行轨迹: {len(points)} 个点, '
            f'关节: {joint_names}')

        result = FollowJointTrajectory.Result()

        # 反馈 lambda
        def feedback_cb(idx, point):
            feedback = FollowJointTrajectory.Feedback()
            feedback.desired = point
            feedback.actual = point        # 无实际反馈源，用期望替代
            goal_handle.publish_feedback(feedback)

        # 取消检查
        def is_canceled():
            return goal_handle.is_cancel_requested

        # 执行插补
        success = self._interpolate_trajectory(
            points, joint_names, servo_client,
            feedback_cb, is_canceled)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = XTrainerBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
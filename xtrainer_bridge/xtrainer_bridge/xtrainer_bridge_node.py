#!/usr/bin/env python3
"""
xtrainer_bridge_node: 双臂桥接节点

功能:
  1. 订阅 Arm1/Arm2 各自命名空间下的 joint_states_robot,
     合并为统一的 /joint_states 话题 (MoveIt 所需)
  2. 为 Arm1/Arm2 各创建一个 FollowJointTrajectory Action Server,
     将 MoveIt 规划的轨迹通过 ServoJ 服务发送至对应机械臂

架构:
  Arm1/dobot_bringup_ros2 ──► joint_states_robot ──┐
                                                    ├──► /joint_states (合并)
  Arm2/dobot_bringup_ros2 ──► joint_states_robot ──┘

  MoveIt ──► /Arm1_controller/follow_joint_trajectory (action) ──► /Arm1/.../ServoJ
  MoveIt ──► /Arm2_controller/follow_joint_trajectory (action) ──► /Arm2/.../ServoJ

轨迹执行策略 (对照官方 dobot_moveit/action_move_server.py):
  - 不做本地插补, 直接按 MoveIt 给的 waypoint 逐点发 ServoJ。
  - t 参数 = 相邻 waypoint 的 time_from_start 差值 (该段运行时间),
    由控制器内部插补器(lookahead_time/gain)做平滑, 避免本地线性插补在
    waypoint 边界产生的速度阶跃/加加速度突变 (卡顿/刹车感的根因)。
  - call_async fire-and-forget: 不等 driver TCP echo 返回, 由绝对时刻节拍
    对齐发送节奏, driver 单线程处理不过来时最多偶发丢点而非堆积突发。
  - 发送前 rad->deg (Dobot 协议要求度)。
"""

import threading
import time
from collections import OrderedDict
from math import degrees

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
        # joint_states 合并发布频率 (Hz), 与 driver 的 JointStatePublishRate 对齐
        self.declare_parameter('joint_state_publish_rate', 50.0)
        # action 超时 (秒)
        self.declare_parameter('action_timeout', 30.0)
        # ServoJ 首点 t 参数 (秒): 第一个 waypoint 没有前驱, 用固定小值。
        self.declare_parameter('servoj_initial_t', 0.05)
        # ServoJ t 下限 (秒): 文档取值范围 [0.004, 3600.0], 下限保护
        self.declare_parameter('servoj_t_min', 0.004)
        # ServoJ 目标角度单位:
        #   'rad' — 从 MoveIt 接收弧度, 发送前转成度 (Dobot 协议要求度)
        #   'deg' — 已是度, 直接发送
        # 驱动发布的 joint_states_robot 是弧度 (command.cpp 把 q_actual deg2Rad),
        # MoveIt 规划也是弧度, 故默认 rad 并在 _send_servoj 转换。
        self.declare_parameter('servoj_angle_unit', 'rad')
        # 兼容旧参数 control_frequency (已废弃, 不再驱动插补; 若传入仅记录告警)
        self.declare_parameter('control_frequency', -1.0)

        self._arm1_joint_names = self.get_parameter('arm1_joint_names').value
        self._arm2_joint_names = self.get_parameter('arm2_joint_names').value
        self._arm1_servo_srv = self.get_parameter('arm1_servoj_service').value
        self._arm2_servo_srv = self.get_parameter('arm2_servoj_service').value
        self._arm1_state_topic = self.get_parameter('arm1_joint_state_topic').value
        self._arm2_state_topic = self.get_parameter('arm2_joint_state_topic').value
        self._js_rate = float(self.get_parameter('joint_state_publish_rate').value)
        self._action_timeout = self.get_parameter('action_timeout').value
        self._servoj_initial_t = float(self.get_parameter('servoj_initial_t').value)
        self._servoj_t_min = float(self.get_parameter('servoj_t_min').value)
        self._servoj_unit = str(self.get_parameter('servoj_angle_unit').value).lower()
        if float(self.get_parameter('control_frequency').value) > 0.0:
            self.get_logger().warn(
                'control_frequency 参数已废弃 (轨迹不再本地插补), '
                'joint_states 频率请用 joint_state_publish_rate')

        self._all_joint_names = self._arm1_joint_names + self._arm2_joint_names

        # ── 关节状态缓存 ──
        self._arm1_positions = OrderedDict((n, 0.0) for n in self._arm1_joint_names)
        self._arm2_positions = OrderedDict((n, 0.0) for n in self._arm2_joint_names)
        self._arm1_lock = threading.Lock()
        self._arm2_lock = threading.Lock()

        # ── 发布者 ──
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── 订阅者 (joint_states 合并转发用独立 MutuallyExclusive group, 避免被轨迹执行抢占) ──
        self._js_cb_group = MutuallyExclusiveCallbackGroup()
        self._arm1_sub = self.create_subscription(
            JointState, self._arm1_state_topic,
            self._arm1_callback, 10, callback_group=self._js_cb_group)
        self._arm2_sub = self.create_subscription(
            JointState, self._arm2_state_topic,
            self._arm2_callback, 10, callback_group=self._js_cb_group)

        # ── ReentrantCallbackGroup (service client + action server 共用) ──
        # 双臂需要并发执行两条轨迹, 必须用 Reentrant + MultiThreadedExecutor。
        # 注意: _pub_timer 不放这里, 否则会被轨迹执行的 time.sleep 抢占线程导致 50Hz 跌到 30Hz。
        self._cb_group = ReentrantCallbackGroup()

        # ── ServoJ 服务客户端 ──
        self._arm1_servo_client = self.create_client(
            ServoJ, self._arm1_servo_srv, callback_group=self._cb_group)
        self._arm2_servo_client = self.create_client(
            ServoJ, self._arm2_servo_srv, callback_group=self._cb_group)

        # ── FollowJointTrajectory Action Server ──
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

        # ── joint_states 发布定时器 (独立 group, 隔离于轨迹执行, 保证稳定 50Hz) ──
        self._pub_timer = self.create_timer(
            1.0 / self._js_rate, self._publish_joint_states,
            callback_group=self._js_cb_group)

        self.get_logger().info('XTrainerBridge 初始化完成')
        self.get_logger().info(f'  左臂关节: {self._arm1_joint_names}')
        self.get_logger().info(f'  右臂关节: {self._arm2_joint_names}')
        self.get_logger().info(f'  Action: /Arm1_controller/follow_joint_trajectory')
        self.get_logger().info(f'  Action: /Arm2_controller/follow_joint_trajectory')
        unit_msg = f'{self._servoj_unit}->deg' if self._servoj_unit == 'rad' else 'deg'
        self.get_logger().info(f'  joint_states: {self._js_rate:.0f}Hz | '
                               f'ServoJ: 直发waypoint, t=相邻dt, 单位={unit_msg}')

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
        """定时发布合并后的 /joint_states (与 driver 同频, 无丢失)"""
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

    # ── 轨迹执行 (对照官方 dobot_moveit/action_move_server.py) ──

    def _execute_trajectory_direct(self, points: list[JointTrajectoryPoint],
                                    servo_client,
                                    is_canceled_cb) -> bool:
        """
        按 MoveIt time_from_start 直发 waypoint, t = 相邻点时间差。

        与官方 action_move_server.py 一致:
          - 不本地插补, 依赖 Dobot 控制器内部 ServoJ 插补器平滑
          - t = next.time_from_start - cur.time_from_start (首点用 servoj_initial_t)
          - 按绝对时刻对齐发送, 抵消 call_async/jitter 偏差
          - call_async fire-and-forget, 不等 TCP echo

        Returns:
            True  — 执行成功完成
            False — 被取消或失败
        """
        if not points:
            return True

        if not servo_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('ServoJ 服务不可用')
            return False

        start_time = time.monotonic()
        prev_tfs = 0.0

        for i, point in enumerate(points):
            if is_canceled_cb():
                self.get_logger().info('轨迹执行被取消')
                return False

            # t = 该段运行时间 = 当前点 time_from_start - 前一点 time_from_start
            cur_tfs = self._time_from_sec_nsec(
                point.time_from_start.sec, point.time_from_start.nanosec)
            if i == 0:
                t = self._servoj_initial_t
            else:
                t = cur_tfs - prev_tfs
            t = max(self._servoj_t_min, min(t, 3600.0))

            # 对齐到该点的绝对时刻 (含 ServoJ 异步发送耗时)
            elapsed = time.monotonic() - start_time
            if elapsed < cur_tfs:
                slack = cur_tfs - elapsed
                if slack > 0:
                    time.sleep(slack)

            self._send_servoj(servo_client, point.positions, t)

            if i % 10 == 0 or i == len(points) - 1:
                self.get_logger().info(
                    f'点{i}/{len(points)-1}: tfs={cur_tfs:.3f}s, '
                    f'实际={time.monotonic()-start_time:.3f}s, t={t:.3f}s')

            prev_tfs = cur_tfs

        return True

    def _send_servoj(self, client, positions: list[float], t: float):
        """
        发送 ServoJ 请求 (fire-and-forget)。

        关键 1 (单位): Dobot CR 协议ServoJ 的 J1~J6 单位是 **度**,
        而 MoveIt 规划是弧度 (ROS REP-103), 必须做 rad->deg 转换。
        关键 2 (t): t 是"该点位的运行时间", 由调用方按相邻 waypoint
        time_from_start 差值传入, 控制器据此做内部插补平滑。
        关键 3 (异步): call_async 不等返回, 由绝对时刻节拍控制发送节奏。
        命令最终形如: ServoJ(J1,...,J6,t=0.0500)  其中 Ji 为度。
        """
        if self._servoj_unit == 'rad':
            target = [degrees(p) for p in positions]
        else:
            target = list(positions)
        req = ServoJ.Request()
        req.a = target[0] if len(target) > 0 else 0.0
        req.b = target[1] if len(target) > 1 else 0.0
        req.c = target[2] if len(target) > 2 else 0.0
        req.d = target[3] if len(target) > 3 else 0.0
        req.e = target[4] if len(target) > 4 else 0.0
        req.f = target[5] if len(target) > 5 else 0.0
        req.param_value = [f't={t:.4f}']
        client.call_async(req)

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
        request = goal_handle.request
        points = request.trajectory.points

        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        self.get_logger().info(
            f'执行轨迹: {len(points)} 个点, 关节: {joint_names}')

        # 取消检查
        def is_canceled():
            return goal_handle.is_cancel_requested

        success = self._execute_trajectory_direct(points, servo_client, is_canceled)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = FollowJointTrajectory.Result()
        if success:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        else:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
        return result


def main(args=None):
    rclpy.init(args=args)
    node = XTrainerBridge()
    executor = MultiThreadedExecutor(num_threads=8)
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

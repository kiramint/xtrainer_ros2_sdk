#!/usr/bin/env python3
"""
xtrainer_controller: 双臂 FollowJointTrajectory → ServoJ 桥接节点

对照官方 dobot_moveit/action_move_server.py 的逻辑:
  - 同步 execute_callback (非 async, 避免 time.sleep 阻塞事件循环)
  - 不本地插补, 直发 MoveIt waypoint
  - t = 相邻 waypoint 的 time_from_start 差值 (首点 t=0.05)
  - 按绝对时刻对齐 (time.sleep(target_tfs - elapsed))
  - call_async fire-and-forget (不等 TCP echo)
  - rad→deg: 180 * j / 3.14159 (照官方)

与官方的区别 (双臂必需):
  - 两个 ActionServer (Arm1/Arm2), 两个 ServoJ client
  - MultiThreadedExecutor + ReentrantCallbackGroup (双臂并发执行)
  官方单臂用 rclpy.spin 单线程即可, 双臂必须并发。
"""

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dobot_msgs_v4.srv import ServoJ


class XTrainerController(Node):

    def __init__(self):
        super().__init__('xtrainer_controller')

        self.declare_parameter('arm1_servoj_service', '/Arm1/dobot_bringup_ros2/srv/ServoJ')
        self.declare_parameter('arm2_servoj_service', '/Arm2/dobot_bringup_ros2/srv/ServoJ')

        self._cb_group = ReentrantCallbackGroup()

        self._arm1_servo = self.create_client(
            ServoJ, self.get_parameter('arm1_servoj_service').value,
            callback_group=self._cb_group)
        self._arm2_servo = self.create_client(
            ServoJ, self.get_parameter('arm2_servoj_service').value,
            callback_group=self._cb_group)

        self._arm1_action = ActionServer(
            self, FollowJointTrajectory, '/Arm1_controller/follow_joint_trajectory',
            execute_callback=self._arm1_execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group)
        self._arm2_action = ActionServer(
            self, FollowJointTrajectory, '/Arm2_controller/follow_joint_trajectory',
            execute_callback=self._arm2_execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group)

        self.get_logger().info('xtrainer_controller 已启动 (双臂 FollowJointTrajectory→ServoJ)')

    def _goal_cb(self, goal_request):
        self.get_logger().info('收到 FollowJointTrajectory goal')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    # ── execute callbacks (同步, MultiThreadedExecutor 分配独立线程) ──

    def _arm1_execute_cb(self, goal_handle):
        return self._execute(goal_handle, self._arm1_servo, 'Arm1')

    def _arm2_execute_cb(self, goal_handle):
        return self._execute(goal_handle, self._arm2_servo, 'Arm2')

    def _execute(self, goal_handle, servo_client, arm_name):
        self.get_logger().info(f'[{arm_name}] Received a new trajectory goal!')
        trajectory = goal_handle.request.trajectory
        points = trajectory.points

        result = FollowJointTrajectory.Result()

        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        if not servo_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'[{arm_name}] ServoJ 服务不可用')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            return result

        self.get_logger().info(
            f'[{arm_name}] 执行轨迹: {len(points)} waypoints, '
            f'Joint Names: {trajectory.joint_names}')

        success = self._execute_trajectory(points, goal_handle, servo_client, arm_name)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info(f'[{arm_name}] 轨迹执行被取消')
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
        elif success:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        else:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED

        return result

    # ── 轨迹执行 (照官方 action_move_server.py) ──

    def _execute_trajectory(self, points, goal_handle, servo_client, arm_name):
        """按 MoveIt time_from_start 直发 waypoint, t = 相邻点时间差。

        - 不本地插补, 依赖 Dobot 控制器内部 ServoJ 插补器平滑
        - t = 相邻 time_from_start 差值 (首点 t=0.05)
        - 按绝对时刻对齐 (time.sleep(cur_tfs - elapsed))
        - call_async fire-and-forget (不等 TCP echo)
        - rad→deg: 180 * j / 3.14159 (照官方)
        """
        start_time = time.monotonic()
        prev_tfs = 0.0

        for i, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                return False

            joint = [180 * p / 3.14159 for p in point.positions]

            cur_tfs = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            t = 0.05 if i == 0 else cur_tfs - prev_tfs
            t = max(0.004, min(t, 3600.0))

            # 对齐到该点的绝对时刻 (含 ServoJ 异步发送耗时)
            elapsed = time.monotonic() - start_time
            if elapsed < cur_tfs:
                slack = cur_tfs - elapsed
                if slack > 0:
                    time.sleep(slack)

            self._servoj_with_t(
                servo_client,
                joint[0], joint[1], joint[2],
                joint[3], joint[4], joint[5],
                t)

            if i % 10 == 0 or i == len(points) - 1:
                actual = time.monotonic() - start_time
                self.get_logger().info(
                    f'[{arm_name}] 点{i}/{len(points)-1}: '
                    f'计划={cur_tfs:.3f}s, 实际={actual:.3f}s, t={t:.3f}s')

            prev_tfs = cur_tfs

        return True

    def _servoj_with_t(self, client, j1, j2, j3, j4, j5, j6, t):
        """带t参数的ServoJ命令 (照官方 ServoJ_C_with_t)"""
        req = ServoJ.Request()
        req.a = float(j1)
        req.b = float(j2)
        req.c = float(j3)
        req.d = float(j4)
        req.e = float(j5)
        req.f = float(j6)
        req.param_value = [f't={t:.4f}']
        client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = XTrainerController()
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

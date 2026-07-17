#!/usr/bin/env python3
"""
xtrainer_controller: 双臂 FollowJointTrajectory → ServoJ 桥接节点

严格对照官方 dobot_moveit/action_move_server.py 的逻辑:
  - async def execute_callback (照官方)
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

    # ── execute callbacks (照官方 async def) ──

    async def _arm1_execute_cb(self, goal_handle):
        return await self._execute(goal_handle, self._arm1_servo, 'Arm1')

    async def _arm2_execute_cb(self, goal_handle):
        return await self._execute(goal_handle, self._arm2_servo, 'Arm2')

    async def _execute(self, goal_handle, servo_client, arm_name):
        self.get_logger().info(f'[{arm_name}] Received a new trajectory goal!')
        trajectory = goal_handle.request.trajectory
        self._execution_trajectory(trajectory, servo_client, arm_name)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    # ── 轨迹执行 (照官方 action_move_server.py 三段式) ──

    def _execution_trajectory(self, trajectory, servo_client, arm_name):
        self.get_logger().info(f'[{arm_name}] Joint Names: {trajectory.joint_names}')

        all_points_with_time = []
        for i, point in enumerate(trajectory.points):
            joint = []
            for ii in point.positions:
                joint.append(180 * ii / 3.14159)  # 弧度转角度 (照官方)

            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            all_points_with_time.append((joint, time_from_start))

            self.get_logger().info(
                f'[{arm_name}] Point {i}: Positions: {joint}, '
                f'TimeFromStart: {time_from_start:.3f}s')

        self._execute_trajectory(all_points_with_time, servo_client, arm_name)

    def _execute_trajectory(self, points_with_time, servo_client, arm_name):
        """按照轨迹时间戳执行轨迹点，动态设置ServoJ的t参数 (照官方)"""
        if not points_with_time:
            return

        self.get_logger().info(f'[{arm_name}] 开始执行轨迹，共{len(points_with_time)}个点')

        start_time = time.time()

        for i, (joint_positions, target_time_from_start) in enumerate(points_with_time):
            if i == 0:
                t = 0.05
            else:
                prev_time = points_with_time[i - 1][1]
                t = target_time_from_start - prev_time

            t = max(0.004, min(t, 3600.0))

            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time < target_time_from_start:
                sleep_time = target_time_from_start - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

            self._servoj_with_t(
                servo_client,
                joint_positions[0], joint_positions[1], joint_positions[2],
                joint_positions[3], joint_positions[4], joint_positions[5],
                t)

            actual_time = time.time() - start_time
            if i % 10 == 0 or i == len(points_with_time) - 1:
                self.get_logger().info(
                    f'[{arm_name}] 点{i}: 计划时间={target_time_from_start:.3f}s, '
                    f'实际时间={actual_time:.3f}s, t={t:.3f}s')

    def _servoj_with_t(self, client, j1, j2, j3, j4, j5, j6, t):
        """带t参数的ServoJ命令 (照官方 ServoJ_C_with_t)"""
        P1 = ServoJ.Request()
        P1.a = float(j1)
        P1.b = float(j2)
        P1.c = float(j3)
        P1.d = float(j4)
        P1.e = float(j5)
        P1.f = float(j6)
        P1.param_value = [f't={t}']
        client.call_async(P1)


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

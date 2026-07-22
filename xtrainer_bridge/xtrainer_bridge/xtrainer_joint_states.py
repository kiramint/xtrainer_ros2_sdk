#!/usr/bin/env python3
"""
xtrainer_joint_states: 双臂 joint_states 合并转发节点

严格对照官方 dobot_moveit/joint_states.py 的事件驱动 passthrough 风格。
区别: 官方是单臂 (订阅 /joint_states_robot 直接转发), 本节点需合并双臂。

策略 (照官方事件驱动, 无 timer):
  - 订阅 /Arm1/joint_states_robot 和 /Arm2/joint_states_robot
  - 收到任一臂消息 → 更新该臂缓存 → 合并 12 关节发布到 /joint_states
  - 发布频率 ≈ 100Hz (两路 50Hz 叠加), 每条都是完整 12 关节
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class XTrainerJointStates(Node):

    def __init__(self):
        super().__init__('xtrainer_joint_states')

        self.declare_parameter('arm1_joint_names', ['J1_1', 'J1_2', 'J1_3', 'J1_4', 'J1_5', 'J1_6'])
        self.declare_parameter('arm2_joint_names', ['J2_1', 'J2_2', 'J2_3', 'J2_4', 'J2_5', 'J2_6'])
        self.declare_parameter('arm1_joint_state_topic', '/Arm1/joint_states_robot')
        self.declare_parameter('arm2_joint_state_topic', '/Arm2/joint_states_robot')

        self._arm1_names = self.get_parameter('arm1_joint_names').value
        self._arm2_names = self.get_parameter('arm2_joint_names').value
        self._all_names = list(self._arm1_names) + list(self._arm2_names)

        self._arm1_pos = [0.0] * len(self._arm1_names)
        self._arm2_pos = [0.0] * len(self._arm2_names)

        self._pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── 哑关节 (URDF 中存在但无驱动的关节, 发布 0 值避免 MoveIt 持续 warning) ──
        self.declare_parameter('dummy_joint_names', [
            'J1_7', 'J1_8',   # Arm1 夹爪手指
            'J2_7', 'J2_8',   # Arm2 夹爪手指
            'J3_1', 'J3_2', 'J3_3', 'J3_4', 'J3_5', 'J3_6',  # Arm3 (未使用)
            'J4_1', 'J4_2', 'J4_3', 'J4_4', 'J4_5', 'J4_6',  # Arm4 (未使用)
        ])
        self._dummy_names = self.get_parameter('dummy_joint_names').value
        self._dummy_pos = [0.0] * len(self._dummy_names)

        self._arm1_sub = self.create_subscription(
            JointState, self.get_parameter('arm1_joint_state_topic').value,
            self._arm1_callback, 10)
        self._arm2_sub = self.create_subscription(
            JointState, self.get_parameter('arm2_joint_state_topic').value,
            self._arm2_callback, 10)

        self.get_logger().info('xtrainer_joint_states 已启动 (事件驱动合并双臂)')

    def _arm1_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._arm1_names:
                self._arm1_pos[self._arm1_names.index(name)] = pos
        self._publish()

    def _arm2_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._arm2_names:
                self._arm2_pos[self._arm2_names.index(name)] = pos
        self._publish()

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = list(self._all_names) + list(self._dummy_names)
        msg.position = list(self._arm1_pos) + list(self._arm2_pos) + list(self._dummy_pos)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = XTrainerJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

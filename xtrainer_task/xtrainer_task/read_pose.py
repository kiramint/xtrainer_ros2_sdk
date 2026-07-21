import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy

from xtrainer_task.robot_move import RobotMover


def main():
    rclpy.init()

    node = rclpy.create_node("read_pose_node")

    moveit = MoveItPy(node_name='xtrainer_task_moveit')
    mover = RobotMover(moveit, node)

    node.get_logger().info('Waiting for MoveIt state to populate …')
    time.sleep(2.0)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            pose1 = mover.get_current_pose('Arm1')
            pose2 = mover.get_current_pose('Arm2')
            node.get_logger().info(
                f'Arm1 — pos:\nx: {pose1.position.x:.4f}\ny: {pose1.position.y:.4f}\nz: {pose1.position.z:.4f}\nox: {pose1.orientation.x:.4f}\noy: {pose1.orientation.y:.4f}\noz: {pose1.orientation.z:.4f}\now: {pose1.orientation.w})'
            )
            node.get_logger().info(
                f'Arm1 — pos:\nx: {pose2.position.x:.4f}\ny: {pose2.position.y:.4f}\nz: {pose2.position.z:.4f}\nox: {pose2.orientation.x:.4f}\noy: {pose2.orientation.y:.4f}\noz: {pose2.orientation.z:.4f}\now: {pose2.orientation.w})'
            )
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

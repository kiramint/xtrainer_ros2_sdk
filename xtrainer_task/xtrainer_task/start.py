"""Entry point for xtrainer_task — robot movement using MoveIt.

Configs (URDF, SRDF, ompl_planning, etc.) are expected to be loaded
from ROS 2 parameters, which are set by the launch file:
    ros2 launch xtrainer_task start.launch.py

Usage
-----
    ros2 launch xtrainer_task start.launch.py
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy

from xtrainer_task.robot_move import RobotMover


def read_current_poses(mover, node):
    for arm in ('Arm1', 'Arm2'):
        try:
            pose = mover.get_current_pose(arm)
            p = pose.position
            q = pose.orientation
            node.get_logger().info(
                f'[{arm}] current pose — '
                f'pos: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}), '
                f'ori: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})'
            )
            return pose
        except Exception as e:
            node.get_logger().error(f'[{arm}] failed to read pose: {e}')
            return Pose()


def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('xtrainer_task_start')
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # MoveItPy reads configs from the parameter server (set by launch file)
    moveit = MoveItPy(node_name='xtrainer_task_moveit')
    mover = RobotMover(moveit, node)

    # Give MoveIt some time to receive latest /joint_states and TF
    node.get_logger().info('Waiting for MoveIt state to populate …')
    time.sleep(2.0)

    # ------------------------------------------------------------------
    # Read current poses
    # ------------------------------------------------------------------
    

    # ------------------------------------------------------------------
    # Spin to keep MoveIt action clients alive
    # ------------------------------------------------------------------
    node.get_logger().info('Spinning (Ctrl-C to exit) …')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
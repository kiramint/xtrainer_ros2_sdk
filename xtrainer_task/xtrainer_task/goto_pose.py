import time

import numpy as nps
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from scipy.spatial.transform import Rotation as R

from xtrainer_task.robot_move import Planner, RobotMover


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

            pose_approach = Pose()
            pose_approach.position.x = 0.2961321175098419
            pose_approach.position.y = -0.15465658903121948
            pose_approach.position.z = 0.6162363290786743
            pose_approach.orientation.x = -0.5022768378257751
            pose_approach.orientation.y = 0.4977162480354309
            pose_approach.orientation.z = -0.5023228526115417
            pose_approach.orientation.w = 0.4976627230644226

            plan_result = mover.plan_pose('Arm1', pose_approach, planner=Planner.ompl)
            mover.execute(plan_result.trajectory)
            
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

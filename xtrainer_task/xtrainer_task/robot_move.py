"""XTrainer robot movement module using MoveIt2 Python API (Jazzy).

Provides plan/execute and current pose query for both Arm1 (left, L1_6)
and Arm2 (right, L2_6).

Configs (URDF, SRDF, ompl_planning, kinematics, etc.) must be provided
via ROS 2 parameters — see launch/start.launch.py for the canonical way.
"""

from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Pose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.node import Node


class RobotMover:
    """High-level interface for dual-arm planning and execution via MoveIt.

    Uses the Jazzy ``moveit_py`` API (``MoveItPy`` + ``PlanningComponent``)
    for Arm1 (left arm, tip L1_6) and Arm2 (right arm, tip L2_6).

    Usage (with launch file providing configs)
    ------------------------------------------
        from xtrainer_task.robot_move import RobotMover
        moveit = MoveItPy(node_name='xtrainer_task_moveit')
        mover = RobotMover(moveit)

        pose = mover.get_current_pose('Arm1')
        plan = mover.plan_joints('Arm1', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        moveit.execute(plan.trajectory, controllers=[])
    """

    # ------------------------------------------------------------------
    _ARM_CONFIG = {
        'Arm1': {
            'group_name': 'Arm1',
            'tip_link': 'L1_6',
        },
        'Arm2': {
            'group_name': 'Arm2',
            'tip_link': 'L2_6',
        },
    }

    def __init__(self, moveit: MoveItPy, node: Node):
        """Construct RobotMover.

        Parameters
        ----------
        moveit : moveit.planning.MoveItPy
            Already-constructed MoveItPy instance (with configs loaded
            from parameters).  If using the launch file the node_name
            should be ``'xtrainer_task_moveit'``.
        node : rclpy.node.Node
            A ROS 2 node used for logging.  The caller is responsible
            for keeping it alive and spinning its executor.
        """
        self._moveit = moveit
        self._node = node
        self._logger = node.get_logger()

        self._planning_scene_monitor = self._moveit.get_planning_scene_monitor()
        self._trajectory_execution_manager = (
            self._moveit.get_trajectory_execution_manager()
        )
        self._robot_model = self._moveit.get_robot_model()

        self._planning_components: dict[str, Any] = {}

        self._logger.info('RobotMover initialised')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_planning_component(self, arm: str):
        cfg = self._ARM_CONFIG.get(arm)
        if cfg is None:
            raise ValueError(
                f"Unknown arm '{arm}'. Valid: {list(self._ARM_CONFIG)}"
            )
        group_name = cfg['group_name']
        if group_name not in self._planning_components:
            self._logger.info(f"Getting planning component '{group_name}' …")
            self._planning_components[group_name] = (
                self._moveit.get_planning_component(group_name)
            )
        return self._planning_components[group_name]

    def _get_tip_link(self, arm: str) -> str:
        return self._ARM_CONFIG[arm]['tip_link']

    # ------------------------------------------------------------------
    # Pose query
    # ------------------------------------------------------------------

    def get_current_pose(self, arm: str) -> Pose:
        """Return the current end-effector pose in the MoveIt world frame.

        The returned pose is expressed in the planning frame
        (``base_link``) and represents the position of the tip link
        (``L1_6`` for Arm1, ``L2_6`` for Arm2).
        """
        tip_link = self._get_tip_link(arm)
        with self._planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            pose = robot_state.get_pose(tip_link)

        self._logger.debug(
            f"[{arm}] pos=({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, "
            f"{pose.position.z:.3f})"
        )
        return pose

    # ------------------------------------------------------------------
    # Planning — joint space
    # ------------------------------------------------------------------

    def plan_joints(
        self,
        arm: str,
        joint_values: List[float],
    ) -> Optional[Any]:
        """Plan a joint-space move for *arm* to *joint_values* (radians).

        Returns the ``PlanResult`` on success (access ``.trajectory``
        on it), or ``None`` on failure.
        """
        if len(joint_values) != 6:
            raise ValueError(
                f"joint_values must have 6 elements, got {len(joint_values)}"
            )
        group_name = self._ARM_CONFIG[arm]['group_name']
        pc = self._get_planning_component(arm)
        pc.set_start_state_to_current_state()

        robot_state = RobotState(self._robot_model)
        robot_state.set_joint_group_positions(group_name, joint_values)
        pc.set_goal_state(robot_state=robot_state)

        self._logger.info(
            f"[{arm}] planning → "
            + ", ".join(f"{v:.3f}" for v in joint_values)
        )
        plan_result = pc.plan()
        if not plan_result:
            self._logger.error(f"[{arm}] planning failed")
            return None

        n_pts = len(plan_result.trajectory.joint_trajectory.points)
        self._logger.info(f"[{arm}] plan OK ({n_pts} waypoints)")
        return plan_result

    # ------------------------------------------------------------------
    # Planning — pose (Cartesian via IK)
    # ------------------------------------------------------------------

    def plan_pose(
        self,
        arm: str,
        target_pose: Pose,
        *,
        frame_id: str = 'base_link',
    ) -> Optional[Any]:
        """Plan a Cartesian move for *arm* to *target_pose*.

        Constructs a PoseStamped, calls ``set_goal_state(pose_stamped_msg=…)``,
        and returns the ``PlanResult`` (or ``None``).
        """
        tip_link = self._get_tip_link(arm)
        pc = self._get_planning_component(arm)
        pc.set_start_state_to_current_state()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = target_pose
        pc.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=tip_link)

        self._logger.info(f"[{arm}] planning pose target (frame: {frame_id}) …")
        plan_result = pc.plan()
        if not plan_result:
            self._logger.error(f"[{arm}] planning failed")
            return None

        n_pts = len(plan_result.trajectory.joint_trajectory.points)
        self._logger.info(f"[{arm}] plan OK ({n_pts} waypoints)")
        return plan_result

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def execute(self, trajectory: Any) -> bool:
        """Execute a trajectory directly via MoveItPy.

        Parameters
        ----------
        trajectory : RobotTrajectory
            The ``.trajectory`` attribute of a PlanResult.

        Returns
        -------
        bool
            ``True`` on success.
        """
        self._logger.info('Executing trajectory …')
        try:
            self._moveit.execute(trajectory, controllers=[])
            self._logger.info('Execution completed')
            return True
        except Exception as e:
            self._logger.error(f'Execution failed: {e}')
            return False

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    def plan_and_execute_joints(
        self,
        arm: str,
        joint_values: List[float],
    ) -> bool:
        """Plan + execute a joint-space move."""
        plan_result = self.plan_joints(arm, joint_values)
        if plan_result is None:
            return False
        return self.execute(plan_result.trajectory)

    def plan_and_execute_pose(
        self,
        arm: str,
        target_pose: Pose,
        *,
        frame_id: str = 'base_link',
    ) -> bool:
        """Plan + execute a Cartesian move."""
        plan_result = self.plan_pose(arm, target_pose, frame_id=frame_id)
        if plan_result is None:
            return False
        return self.execute(plan_result.trajectory)
"""XTrainer robot movement module using MoveIt2 Python API (Jazzy).

Provides plan/execute and current pose query for both Arm1 (left, L1_6)
and Arm2 (right, L2_6).

Configs (URDF, SRDF, ompl_planning, kinematics, etc.) must be provided
via ROS 2 parameters — see launch/start.launch.py for the canonical way.
"""

from enum import Enum
from typing import Any, List, Optional

from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node


class Planner(Enum):
    """Supported planner identifiers — pass to any ``plan_*()`` method.

    Each member carries ``planning_pipeline`` and ``planner_id`` for
    documentation. Its enum name also matches a parameter namespace in
    ``config/moveit_cpp.yaml`` used to build ``PlanRequestParameters``.
    """

    ompl = ('ompl', 'RRTConnectkConfigDefault', 'plan_request_params')
    pilz_ptp = ('pilz_industrial_motion_planner', 'PTP', 'pilz_ptp')
    pilz_lin = ('pilz_industrial_motion_planner', 'LIN', 'pilz_lin')
    pilz_cap_ptp = (
        'pilz_industrial_motion_planner', 'PTP', 'pilz_cap_ptp'
    )

    def __init__(
        self,
        planning_pipeline: str,
        planner_id: str,
        parameter_namespace: str,
    ):
        self.planning_pipeline = planning_pipeline
        self.planner_id = planner_id
        self.parameter_namespace = parameter_namespace


class RobotMover:
    """High-level interface for dual-arm planning and execution via MoveIt.

    Uses the Jazzy ``moveit_py`` API (``MoveItPy`` + ``PlanningComponent``)
    for Arm1 (left arm, tip L1_6) and Arm2 (right arm, tip L2_6).

    Planner selection
    -----------------
    Every ``plan_*()`` / ``plan_and_execute_*()`` method accepts a
    ``planner`` keyword argument (default ``'ompl'``):

    | ``planner``    | pipeline                          | planner_id                  |
    |----------------|-----------------------------------|-----------------------------|
    | ``'ompl'``     | ompl                              | RRTConnectkConfigDefault    |
    | ``'pilz_ptp'`` | pilz_industrial_motion_planner    | PTP  (point-to-point/joint) |
    | ``'pilz_lin'`` | pilz_industrial_motion_planner    | LIN  (linear/Cartesian)     |

    Usage (with launch file providing configs)
    ------------------------------------------
        from xtrainer_task.robot_move import RobotMover
        moveit = MoveItPy(node_name='xtrainer_task_moveit')
        mover = RobotMover(moveit, node)

        pose = mover.get_current_pose('Arm1')
        plan = mover.plan_joints('Arm1', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Use Pilz PTP:
        plan = mover.plan_joints('Arm1', [...], planner='pilz_ptp')
        # Use Pilz LIN for a Cartesian pose:
        plan = mover.plan_pose('Arm1', target_pose, planner='pilz_lin')
        moveit.execute(plan.trajectory, controllers=[])
    """

    # ------------------------------------------------------------------
    _ARM_CONFIG = {
        'Arm1': {
            'group_name': 'Arm1',
            'tip_link': 'L1_gripper_tcp',  # backward-compatible default
        },
        'Arm2': {
            'group_name': 'Arm2',
            'tip_link': 'L2_gripper_tcp',
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

        self._display_pub = node.create_publisher(
            DisplayTrajectory, '/display_planned_path', 1
        )

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

    def _get_tip_link(self, arm: str, tip_link: Optional[str] = None) -> str:
        """Return the configured tip, or an explicitly selected link.

        ``None`` preserves the historical TCP behavior.  The new grasp tips
        can be selected with ``tip_link='L1_gripper_tip'`` or
        ``tip_link='L2_gripper_tip'`` on pose-related APIs.
        """
        if arm not in self._ARM_CONFIG:
            raise ValueError(f"Unknown arm '{arm}'")
        return tip_link or self._ARM_CONFIG[arm]['tip_link']

    @staticmethod
    def _resolve_planner(planner: 'Planner | str') -> Optional['Planner']:
        """Validate *planner* and return the :class:`Planner` enum member.

        Returns ``None`` if *planner* is the MoveItPy default
        (``Planner.ompl``, which means "use whatever ``plan_request_params``
        says"), so callers can skip overriding.
        """
        if isinstance(planner, str):
            try:
                planner = Planner[planner]
            except KeyError as exc:
                valid = ', '.join(member.name for member in Planner)
                raise ValueError(
                    f"Unknown planner '{planner}'. Valid: {valid}"
                ) from exc
        if not isinstance(planner, Planner):
            raise TypeError('planner must be a Planner or planner name string')
        if planner is Planner.ompl:
            return None
        return planner

    def _plan(self, pc: Any, planner: 'Planner | str') -> Any:
        """Plan with the default config or a named request parameter set."""
        planner_cfg = self._resolve_planner(planner)
        if planner_cfg is None:
            return pc.plan()

        plan_parameters = PlanRequestParameters(
            self._moveit, planner_cfg.parameter_namespace
        )
        return pc.plan(single_plan_parameters=plan_parameters)

    def _display_trajectory(self, trajectory):
        msg = DisplayTrajectory()
        msg.model_id = self._robot_model.name
        msg.trajectory.append(trajectory.get_robot_trajectory_msg())
        self._display_pub.publish(msg)

    # ------------------------------------------------------------------
    # Pose query
    # ------------------------------------------------------------------

    def get_current_pose(
        self, arm: str, *, tip_link: Optional[str] = None
    ) -> Pose:
        """Return the current end-effector pose in the MoveIt world frame.

        The returned pose is expressed in the planning frame
        (``base_link``) and represents the position of the tip link
        (the configured TCP by default).  Pass ``tip_link`` to query another
        fixed link, for example ``L1_gripper_tip``.
        """
        selected_tip = self._get_tip_link(arm, tip_link)
        with self._planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            pose = robot_state.get_pose(selected_tip)

        self._logger.debug(
            f"[{arm}] pos=({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, "
            f"{pose.position.z:.3f})"
        )
        return pose
    
    # ------------------------------------------------------------------
    # Planning — named target (SRDF group_state)
    # ------------------------------------------------------------------

    def plan_named(
        self,
        arm: str,
        target_name: str,
        *,
        planner: Planner | str = Planner.ompl,
    ) -> Optional[Any]:
        """Plan a move for *arm* to a predefined named target in the SRDF.

        Named targets are ``<group_state>`` entries defined in the SRDF
        under the group corresponding to *arm* (e.g. ``"Home1"`` for Arm1).

        Parameters
        ----------
        arm : str
            ``'Arm1'`` or ``'Arm2'``.
        target_name : str
            Name of the ``<group_state>`` in the SRDF (e.g. ``"Home1"``).
        planner : Planner | str
            Planner to use (default :attr:`Planner.ompl`).

        Returns
        -------
        PlanResult or None
        """
        pc = self._get_planning_component(arm)
        pc.set_start_state_to_current_state()

        # Validate target exists
        named_states = pc.named_target_states
        if target_name not in named_states:
            self._logger.error(
                f"[{arm}] unknown named target '{target_name}'. "
                f"Available: {named_states}"
            )
            return None

        # Set goal from named target joint values
        joint_dict = pc.get_named_target_state_values(target_name)
        import numpy as np
        joint_values = np.array([joint_dict[jn] for jn in joint_dict])
        group_name = self._ARM_CONFIG[arm]['group_name']
        robot_state = RobotState(self._robot_model)
        robot_state.set_joint_group_positions(group_name, joint_values)
        pc.set_goal_state(robot_state=robot_state)

        self._logger.info(
            f"[{arm}] planning to named target '{target_name}' "
            f"(planner={planner.name if isinstance(planner, Planner) else planner}) …"
        )
        plan_result = self._plan(pc, planner)
        if not plan_result:
            self._logger.error(
                f"[{arm}] planning to '{target_name}' failed"
            )
            return None

        n_pts = len(plan_result.trajectory)
        self._logger.info(f"[{arm}] plan OK ({n_pts} waypoints)")
        self._display_trajectory(plan_result.trajectory)
        return plan_result

    # ------------------------------------------------------------------
    # Planning — relative single-joint motion
    # ------------------------------------------------------------------

    def get_current_joint_positions(self, arm: str) -> List[float]:
        """Return the current active joint positions for *arm* in radians."""
        group_name = self._ARM_CONFIG[arm]['group_name']
        with self._planning_scene_monitor.read_only() as scene:
            positions = scene.current_state.get_joint_group_positions(
                group_name
            )
        return [float(value) for value in positions]

    def plan_joint_delta(
        self,
        arm: str,
        joint_name: str,
        delta_rad: float,
        *,
        planner: Planner | str = Planner.pilz_ptp,
    ) -> Optional[Any]:
        """Plan a relative move that changes only one active joint.

        The target is based on the latest PlanningScene joint state. MoveIt
        validates the resulting target against the planning group's bounds.
        """
        group_name = self._ARM_CONFIG[arm]['group_name']
        joint_group = self._robot_model.get_joint_model_group(group_name)
        joint_names = list(joint_group.active_joint_model_names)

        if joint_name not in joint_names:
            raise ValueError(
                f"Joint '{joint_name}' is not active in {arm}. "
                f"Valid: {joint_names}"
            )

        current = self.get_current_joint_positions(arm)
        if len(current) != len(joint_names):
            raise RuntimeError(
                f"[{arm}] expected {len(joint_names)} joint positions, "
                f"got {len(current)}"
            )

        target = current.copy()
        joint_index = joint_names.index(joint_name)
        target[joint_index] += float(delta_rad)

        bounds = list(joint_group.active_joint_model_bounds)
        target_bound = bounds[joint_index][0]
        if (
            target_bound.position_bounded
            and not target_bound.min_position
            <= target[joint_index]
            <= target_bound.max_position
        ):
            raise ValueError(
                f"[{arm}] {joint_name} target {target[joint_index]:.3f} rad "
                f"is outside [{target_bound.min_position:.3f}, "
                f"{target_bound.max_position:.3f}] rad"
            )

        self._logger.info(
            f"[{arm}] relative joint move {joint_name}: "
            f"{current[joint_index]:.3f} + {delta_rad:.3f} = "
            f"{target[joint_index]:.3f} rad"
        )
        return self.plan_joints(arm, target, planner=planner)

    # ------------------------------------------------------------------
    # Planning — joint space
    # ------------------------------------------------------------------

    def plan_joints(
        self,
        arm: str,
        joint_values: List[float],
        *,
        planner: Planner | str = Planner.ompl,
    ) -> Optional[Any]:
        """Plan a joint-space move for *arm* to *joint_values* (radians).

        Parameters
        ----------
        arm : str
            ``'Arm1'`` or ``'Arm2'``.
        joint_values : list[float]
            Six joint angles in radians.
        planner : Planner | str
            Planner to use (default :attr:`Planner.ompl`).
            Also accepts string values ``'ompl'``, ``'pilz_ptp'``,
            ``'pilz_lin'``.

        Returns
        -------
        PlanResult or None
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

        planner_name = (
            planner.name if isinstance(planner, Planner) else planner
        )
        self._logger.info(
            f"[{arm}] planning (planner={planner_name}) → "
            + ", ".join(f"{v:.3f}" for v in joint_values)
        )
        plan_result = self._plan(pc, planner)
        if not plan_result:
            self._logger.error(f"[{arm}] planning failed")
            return None

        n_pts = len(plan_result.trajectory)
        self._logger.info(f"[{arm}] plan OK ({n_pts} waypoints)")
        self._display_trajectory(plan_result.trajectory)
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
        planner: Planner | str = Planner.ompl,
        tip_link: Optional[str] = None,
    ) -> Optional[Any]:
        """Plan a Cartesian move for *arm* to *target_pose*.

        Constructs a PoseStamped, calls ``set_goal_state(pose_stamped_msg=…)``,
        and returns the ``PlanResult`` (or ``None``).

        Parameters
        ----------
        arm : str
            ``'Arm1'`` or ``'Arm2'``.
        target_pose : Pose
            Target end-effector pose.
        frame_id : str
            Frame in which *target_pose* is expressed (default ``'base_link'``).
        planner : Planner | str
            Planner to use (default :attr:`Planner.ompl`).
            ``Planner.pilz_lin`` is the natural choice for Cartesian
            linear moves.

        Returns
        -------
        PlanResult or None
        """
        selected_tip = self._get_tip_link(arm, tip_link)
        pc = self._get_planning_component(arm)
        pc.set_start_state_to_current_state()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = target_pose
        pc.set_goal_state(
            pose_stamped_msg=pose_stamped, pose_link=selected_tip
        )

        planner_name = (
            planner.name if isinstance(planner, Planner) else planner
        )
        self._logger.info(
            f"[{arm}] planning pose target "
            f"(frame: {frame_id}, planner={planner_name}) …"
        )
        plan_result = self._plan(pc, planner)
        if not plan_result:
            self._logger.error(f"[{arm}] planning failed")
            return None

        n_pts = len(plan_result.trajectory)
        self._logger.info(f"[{arm}] plan OK ({n_pts} waypoints)")
        self._display_trajectory(plan_result.trajectory)
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
        *,
        planner: Planner | str = Planner.ompl,
    ) -> bool:
        """Plan + execute a joint-space move.

        Parameters
        ----------
        planner : Planner | str
            Forwarded to :meth:`plan_joints`.
        """
        plan_result = self.plan_joints(arm, joint_values, planner=planner)
        if plan_result is None:
            return False
        return self.execute(plan_result.trajectory)

    def plan_and_execute_pose(
        self,
        arm: str,
        target_pose: Pose,
        *,
        frame_id: str = 'base_link',
        planner: Planner | str = Planner.ompl,
        tip_link: Optional[str] = None,
    ) -> bool:
        """Plan + execute a Cartesian move.

        Parameters
        ----------
        planner : Planner | str
            Forwarded to :meth:`plan_pose`.
        """
        plan_result = self.plan_pose(
            arm, target_pose, frame_id=frame_id, planner=planner,
            tip_link=tip_link,
        )
        if plan_result is None:
            return False
        return self.execute(plan_result.trajectory)

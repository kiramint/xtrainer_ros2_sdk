"""不启动 ROS 的几何检查。"""

from pathlib import Path

import numpy as np

from xtrainer_mcp.geometry import (
    UrdfKinematics,
    median_depth_mm,
    parse_named_states,
    select_indices,
    trajectory_json,
    unproject,
    validate_servo_path,
)

_ROOT = Path(__file__).resolve().parents[2]
_URDF = _ROOT / "xtrainer_description" / "urdf" / "x_trainer.urdf"
_SRDF = _ROOT / "xtrainer_moveit" / "config" / "x_trainer.srdf"


def test_unproject_optical_axes():
    # 主点、深度 1 m → 光轴上 (0, 0, 1)
    assert unproject(320, 240, 1.0, 500, 400, 320, 240) == (0.0, 0.0, 1.0)
    # u 比主点大一个焦距 → X = 深度
    x, y, z = unproject(820, 240, 2.0, 500, 400, 320, 240)
    assert abs(x - 2.0) < 1e-9
    assert y == 0.0 and z == 2.0
    # v 比主点大一个焦距 → Y 向下为正
    x, y, z = unproject(320, 640, 2.0, 500, 400, 320, 240)
    assert x == 0.0
    assert abs(y - 2.0) < 1e-9


def test_median_depth_ignores_zeros():
    depth = np.zeros((10, 10), dtype=np.uint16)
    depth[4:7, 4:7] = np.array(
        [
            [0, 100, 0],
            [300, 0, 500],
            [0, 700, 0],
        ],
        dtype=np.uint16,
    )
    assert median_depth_mm(depth, 5, 5, window=3) == 400.0
    assert median_depth_mm(depth, 0, 0, window=1) is None
    assert median_depth_mm(depth, 20, 5, window=5) is None


def test_trajectory_json_keeps_endpoints_and_reports_original_count():
    count = 250
    times = [i * 0.01 for i in range(count)]
    positions = [[float(i), 0.0] for i in range(count)]
    body = trajectory_json(["J1_1", "J1_2"], times, positions, max_points=200)
    assert body["point_count"] == 250
    assert body["returned_point_count"] <= 200
    assert body["points"][0]["t"] == 0.0
    assert body["points"][0]["positions"][0] == 0.0
    assert body["points"][-1]["positions"][0] == 249.0
    assert body["duration_sec"] == 2.49
    indices = select_indices(250, 200)
    assert indices[0] == 0 and indices[-1] == 249


def test_servo_limits():
    current = [0.0] * 6
    assert validate_servo_path(current, [[0.1] * 6], 0.05) is None
    assert "最多" in validate_servo_path(current, [[0.0] * 6] * 61, 0.02)
    assert "dt" in validate_servo_path(current, [[0.0] * 6], 0.01)
    assert "总时长" in validate_servo_path(current, [[0.0] * 6] * 60, 0.1)
    assert "单步" in validate_servo_path(current, [[0.2, 0, 0, 0, 0, 0]], 0.05)
    assert validate_servo_path(current, [], 0.05) is not None


def test_synthetic_revolute_moves_the_tip():
    urdf = """
    <robot name="toy">
      <link name="base_link"/>
      <link name="mid"/>
      <link name="tip"/>
      <joint name="spin" type="revolute">
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="mid"/>
        <axis xyz="0 0 1"/>
      </joint>
      <joint name="reach" type="fixed">
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <parent link="mid"/>
        <child link="tip"/>
      </joint>
    </robot>
    """
    kin = UrdfKinematics(urdf)
    rest = kin.link_pose("tip", {"spin": 0.0})
    turned = kin.link_pose("tip", {"spin": 1.57079632679})
    assert abs(rest["position"]["x"] - 1.0) < 1e-6
    assert abs(rest["position"]["y"]) < 1e-6
    assert abs(turned["position"]["x"]) < 1e-5
    assert abs(turned["position"]["y"] - 1.0) < 1e-5


def test_depth_png_keeps_millimetres():
    import cv2

    depth = np.array([[0, 1000], [2000, 65535]], dtype=np.uint16)
    ok, encoded = cv2.imencode(".png", depth)
    assert ok
    decoded = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    assert decoded.dtype == np.uint16
    assert int(decoded[0, 1]) == 1000
    assert int(decoded[1, 0]) == 2000


def test_real_urdf_tip_offset_and_home():
    kin = UrdfKinematics(_URDF.read_text())
    wrist = kin.link_transform("L1_6", {})
    tip = kin.link_transform("L1_gripper_tip", {})
    tcp = kin.link_transform("L1_gripper_tcp", {})
    assert abs(np.linalg.norm(tip[:3, 3] - wrist[:3, 3]) - 0.195) < 1e-6
    assert abs(np.linalg.norm(tcp[:3, 3] - wrist[:3, 3]) - 0.14568) < 1e-6
    states = parse_named_states(_SRDF.read_text())
    assert states[("Arm1", "Home1")]["J1_1"] == 0.0
    assert states[("Arm2", "Home2")]["J2_6"] == 0.0
    assert len(states[("Arm1", "Home1")]) == 6

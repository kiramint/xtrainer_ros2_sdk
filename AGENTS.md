# AGENTS.md — XTrainer 项目记忆

> 最后更新: 2026-07-14

---

## 项目概述

XTrainer 是一个双臂机器人系统，使用 ROS2 进行控制。

### 硬件配置

| 组件 | 数量 | 说明 |
|------|------|------|
| Dobot 机械臂 | 4 | Arm1(左)/Arm2(右) 各 2 个一组 |
| RealSense 相机 | 3 | 头顶(camera_top)、左手(camera_left)、右手(camera_right) |
| 夹爪 | 2 | 串口伺服夹爪 |

### 软件包结构

| 包名 | 用途 |
|------|------|
| `dobot_bringup_v4` | 单个机械臂驱动节点 (`cr_robot_ros2_node`) |
| `xtrainer_bridge` | 合并双臂 joint_states → `/joint_states`，提供 FollowJointTrajectory Action |
| `xtrainer_control` | 顶层启动与标定 launch 文件 |
| `xtrainer_description` | URDF 模型 (x_trainer.urdf) |
| `xtrainer_gripper` | 串口夹爪控制节点 |
| `easy_handeye2` | 手眼标定工具包 |

---

## URDF Frame 命名约定

### 基座

- `base_link` — 机器人根坐标系 (通过 `root_joint` 固定于 `World`)

### Arm1 (左臂)

| 关节 | Link | 说明 |
|------|------|------|
| J1_1 ~ J1_6 | L1_1 ~ L1_6 | 6 个旋转关节 |
| J1_7, J1_8 | L1_7, L1_8 | 2 个 prismatic 关节 (夹爪手指) |
| J1_gripper_tcp (fixed) | L1_gripper_tcp | 末端 TCP |

- 末端 effector frame: `L1_6`

### Arm2 (右臂)

| 关节 | Link | 说明 |
|------|------|------|
| J2_1 ~ J2_6 | L2_1 ~ L2_6 | 6 个旋转关节 |
| J2_7, J2_8 | L2_7, L2_8 | 2 个 prismatic 关节 (夹爪手指) |
| J2_gripper_tcp (fixed) | L2_gripper_tcp | 末端 TCP |

- 末端 effector frame: `L2_6`

> **注意**: Arm3(J3_*) 和 Arm4(J4_*) 也存在于 URDF 中，但目前未使用。

---

## 命名空间与话题约定

### 手臂节点

- Arm1 节点: namespace `/Arm1`，发布 `joint_states_robot`
- Arm2 节点: namespace `/Arm2`，发布 `joint_states_robot`
- xtrainer_bridge 合并后发布 `/joint_states` (frame_id: `base_link`)

### 相机话题

所有相机使用 namespace `camera`，通过 `camera_name` 区分子话题：

| 相机 | camera_name | 彩色话题路径 |
|------|-------------|-------------|
| 头顶 | camera_top | `/camera/camera_top/color/image_raw` |
| 左手 | camera_left | `/camera/camera_left/color/image_raw` |
| 右手 | camera_right | `/camera/camera_right/color/image_raw` |

---

## Launch 文件结构

### `start.launch.py` — 主启动文件

启动以下组件:

1. `xtrainer_driver` (dobot_bringup_v4 → xtrainer.launch.py: 双臂节点 + robot_state_publisher + xtrainer_bridge)
2. `gripper_1` + `gripper_2` (夹爪节点)
3. `realsense_camera_top` + `realsense_camera_left` + `realsense_camera_right`
4. 按需加载 easy_handeye2 标定结果 (publish.launch.py)

### 标定启动文件

| 文件 | 相机 | 标定类型 | 标定名 | effector |
|------|------|----------|--------|----------|
| `calibrate_top.launch.py` | camera_top | `eye_on_base` | `top_cam_cal` | L1_6 |
| `calibrate_left.launch.py` | camera_left | `eye_in_hand` | `left_cam_cal` | L1_6 |
| `calibrate_right.launch.py` | camera_right | `eye_in_hand` | `right_cam_cal` | L2_6 |

标定产物路径: `~/.ros2/easy_handeye2/calibrations/<name>.calib`

easy_handeye2 标定参数:

- `calibration_type`: `eye_on_base` 或 `eye_in_hand`
- `name`: 标定唯一名称，对应 .calib 文件名
- `robot_base_frame`: `base_link`
- `robot_effector_frame`: 机械臂末端 link
- `tracking_base_frame`: `camera_base`
- `tracking_marker_frame`: `camera_marker`

### ArUco 标定板参数

- marker_id: 99
- marker_size: 0.10 (10cm)
- corner_refinement: LINES

---

## 已知待配置项

以下配置使用占位符，需要在部署时填入实际值：

- 各相机 `serial_no`: 当前为空字符串 `""`
- 夹爪 `port`: `/dev/ttyUSB0`、`/dev/ttyUSB1`
- 相机 `camera_frame`: 当前为 `"rgb_camera_link"`，需要与 realsense 实际发布的 TF frame 名称对齐

---

## 本次会话修改记录

### `calibrate_top.launch.py`

- [x] 修正 aruco topic: `"/rgb/..."` (TODO) → `"/camera/camera_top/color/..."`
- [x] 修正标定名: `"right_arm_cal"` → `"top_cam_cal"`
- [x] 添加缺失的 realsense 相机节点
- [x] 清理未使用的 import (`DeclareLaunchArgument`, `LaunchConfiguration`)

### `start.launch.py`

- [x] 清理未使用的 import (`DeclareLaunchArgument`, `LaunchConfiguration`)

### 新增文件

- `calibrate_left.launch.py` — 左手相机 eye_in_hand 标定
- `calibrate_right.launch.py` — 右手相机 eye_in_hand 标定

### `setup.py`

- [x] 注册 `calibrate_left.launch.py` 和 `calibrate_right.launch.py`

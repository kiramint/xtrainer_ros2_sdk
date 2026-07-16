# AGENTS.md — XTrainer 项目记忆

> 最后更新: 2026-07-16

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
| `xtrainer_control` | 顶层启动、标定、机械臂状态控制 |
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
5. 延迟 15s 后自动使能双臂 (enable_arms)

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

## 机械臂上电与使能

### Dobot 协议要点

- `PowerOn()` — 上电 (立即返回)，约需 **10 秒** 完成初始化
- `EnableRobot()` — 使能 (必须在 PowerOn 完成、RobotMode ≥ 4 后调用)
- `DisableRobot()` — 去使能
- `StartDrag()` — 进入拖拽示教模式
- `StopDrag()` — 退出拖拽示教模式
- `RobotMode()` 返回值: 1=初始化, 2=抱闸松开, 3=未上电, 4=未使能, 5=使能空闲, 6=拖拽模式, 7=运行中, 9=报错

### 控制库 `robot_control.py`

`xtrainer_control.robot_control` 模块提供以下函数：

| 函数 | 说明 | 参数 |
|------|------|------|
| `enable_arm(node, ns)` | 单臂使能 (PowerOn→轮询RobotMode→EnableRobot)，已使能则跳过 | node, arm_namespace, timeout=30s |
| `disable_arm(node, ns)` | 单臂去使能 (DisableRobot) | node, arm_namespace, timeout=10s |
| `start_drag(node, ns)` | 单臂拖拽示教开 | node, arm_namespace, timeout=5s |
| `stop_drag(node, ns)` | 单臂拖拽示教关 | node, arm_namespace, timeout=5s |
| `enable_all(node, nss)` | 多臂使能，默认 `["Arm1","Arm2"]` | node, namespaces, timeout=10s |
| `disable_all(node, nss)` | 多臂去使能 | node, namespaces, timeout=10s |

### 控制 launch 文件

| 文件 | 功能 |
|------|------|
| `launch/enable.launch.py` | 一键使能双臂 |
| `launch/disable.launch.py` | 一键去使能双臂 |
| `launch/enable_and_drag.launch.py` | 一键使能+示教双臂 |

### 手动调用示例

```bash
ros2 launch xtrainer_control enable.launch.py
ros2 launch xtrainer_control disable.launch.py
ros2 run xtrainer_control enable_arms
ros2 run xtrainer_control disable_arms
```

---

## 本次会话修改记录

### `dobot_bringup_v4` — 删除 robot_number，修复 namespace bug

- [x] **根因**: `kRobotName` 为空时拼接 `/dobot_bringup_ros2/...`（绝对路径），导致 namespace 失效
- [x] 删除 `cr_robot_ros2.h` 中的 `kRobotName` 成员变量
- [x] 删除 `cr_robot_ros2.cpp` 中的 `robot_number` 参数和 `kRobotName` 逻辑
- [x] 所有服务/topic 路径改为相对路径（如 `dobot_bringup_ros2/srv/EnableRobot`）
- [x] 删除 `xtrainer.launch.py` / `dobot_bringup_ros2.launch.py` 中的 `robot_number` 参数
- [x] 删除 `param.json` 中的 `robot_number` 和 `current_robot` 字段
- [x] 修复 `dobot_bringup_ros2.launch.py` 对 `current_robot` 的引用

### `xtrainer_control` — 新增机械臂状态控制

- [x] 新增 `robot_control.py` — Python 控制库
- [x] 新增 `enable_arms.py` / `disable_arms.py` / `enable_and_drag.py` — ROS2 入口脚本
- [x] 新增 `launch/enable.launch.py` / `launch/disable.launch.py` / `launch/enable_and_drag.launch.py`
- [x] 更新 `setup.py` — 注册入口点和 launch 文件

### `robot_control.py` — 时序修复

- [x] **根因**: `PowerOn` 异步，需等约 10s 初始化完成才能调 `EnableRobot`
- [x] 增大 `enable_arm` 默认超时到 30s
- [x] PowerOn 后通过 `RobotMode` 轮询等待（每秒查询，直到 mode ≥ 4 或 mode = 9 报错）
- [x] enable 前先查 RobotMode，若 mode ≥ 5 则跳过（幂等）

### `start.launch.py` — 自动使能

- [x] 使用 `TimerAction(period=15.0)` 延迟 15s 后自动执行 `enable_arms`
- [x] 确保驱动节点先启动、TCP 连接建立后再使能

### `calibrate_top.launch.py` (上次会话)

- [x] 修正 aruco topic: `"/rgb/..."` (TODO) → `"/camera/camera_top/color/..."`
- [x] 修正标定名: `"right_arm_cal"` → `"top_cam_cal"`
- [x] 添加缺失的 realsense 相机节点
- [x] 清理未使用的 import (`DeclareLaunchArgument`, `LaunchConfiguration`)

### `start.launch.py` (上次会话)

- [x] 清理未使用的 import (`DeclareLaunchArgument`, `LaunchConfiguration`)

### 新增文件 (上次会话)

- `calibrate_left.launch.py` — 左手相机 eye_in_hand 标定
- `calibrate_right.launch.py` — 右手相机 eye_in_hand 标定

### `setup.py` (上次会话)

- [x] 注册 `calibrate_left.launch.py` 和 `calibrate_right.launch.py`

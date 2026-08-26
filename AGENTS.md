# AGENTS.md — XTrainer 项目记忆

> 最后更新: 2026-08-02

---

## 项目概述

XTrainer 是一个双臂机器人系统，使用 ROS2 Jazzy + Ubuntu24.04 进行控制。

### 硬件配置

| 组件 | 数量 | 说明 |
| ------ | ------ | ------ |
| Dobot 机械臂 | 4 | Arm1(左)/Arm2(右) 各 2 个一组，型号 **Nova2** |
| RealSense 相机 | 3 | 头顶(camera_top)、左手(camera_left)、右手(camera_right) |
| 夹爪 | 2 | 串口伺服夹爪 |

### 软件包结构

| 包名 | 用途 |
| ------ | ------ |
| `dobot_bringup_v4` | 单个机械臂驱动节点 (`cr_robot_ros2_node`) |
| `xtrainer_bridge` | 双臂桥接: `xtrainer_joint_states` (合并 joint_states) + `xtrainer_controller` (FollowJointTrajectory→ServoJ) |
| `xtrainer_control` | 顶层启动、标定、机械臂状态控制 |
| `xtrainer_description` | URDF 模型 (x_trainer.urdf) |
| `xtrainer_gripper` | 串口夹爪控制节点 |
| `xtrainer_task` | MoveItPy 任务层控制 (Python API, 规划+执行+可视化) |
| `easy_handeye2` | 手眼标定工具包 |

---

## URDF Frame 命名约定

### 基座

- `base_link` — 机器人根坐标系 (通过 `root_joint` 固定于 `World`)

### Arm1 (左臂)

| 关节 | Link | 说明 |
| ------ | ------ | ------ |
| J1_1 ~ J1_6 | L1_1 ~ L1_6 | 6 个旋转关节 |
| J1_7, J1_8 | L1_7, L1_8 | 2 个 prismatic 关节 (夹爪手指) |
| J1_gripper_tcp (fixed) | L1_gripper_tcp | 末端 TCP |
| J1_gripper_tip (fixed) | L1_gripper_tip | 抓取尖端，沿 L1_6 局部 Z 轴 0.195 m |

- 末端 effector frame: `L1_gripper_tcp`
- 默认 effector frame: `L1_gripper_tcp` (保持兼容)
- 抓取 tip frame: `L1_gripper_tip` (当前实际长度 19.5 cm)

### Arm2 (右臂)

| 关节 | Link | 说明 |
| ------ | ------ | ------ |
| J2_1 ~ J2_6 | L2_1 ~ L2_6 | 6 个旋转关节 |
| J2_7, J2_8 | L2_7, L2_8 | 2 个 prismatic 关节 (夹爪手指) |
| J2_gripper_tcp (fixed) | L2_gripper_tcp | 末端 TCP |
| J2_gripper_tip (fixed) | L2_gripper_tip | 抓取尖端，沿 L2_6 局部 Z 轴 0.195 m |

- 末端 effector frame: `L2_gripper_tcp`
- 默认 effector frame: `L2_gripper_tcp` (保持兼容)
- 抓取 tip frame: `L2_gripper_tip` (当前实际长度 19.5 cm)

> **Tip link 注意**: `L1_gripper_tip` / `L2_gripper_tip` 在
> `xtrainer_description/urdf/x_trainer.urdf` 中通过 fixed joint 定义，当前
> `<origin xyz="0 0 0.195"/>`。它们是用于精确抓取定位的临时物理尖端模型，
> 与原有 TCP 并存，不替换默认 MoveIt effector。

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

**点云话题** (2026-08-04 起由 `depth_image_proc/point_cloud_xyzrgb_node` 发布, 非 realsense 自带):

| 话题 | frame_id | H×W | 说明 |
|------|----------|-----|------|
| `/camera/<X>/aligned_depth_to_color/image_raw` | `camera_X_color_optical_frame` | 1280×720 | realsense 发, 带全套滤波 (decimation/spatial/temporal/hole_filling) |
| `/camera/<X>/depth/color/points` | `camera_X_color_optical_frame` | 720×1280 | depth_image_proc 发, **color 视角**, SAM mask 可 1:1 直接作用 |

> **不要再用 realsense 的 `pointcloud.enable`**: 该路径在 depth 坐标系 (frame_id=`depth_optical_frame`, H×W=decimation 后 depth 分辨率), 与 SAM mask 错位。详见本次会话记录 (2026-08-04)。

---

## Launch 文件结构

### `start.launch.py` — 主启动文件

启动以下组件:

1. `xtrainer_driver` (dobot_bringup_v4 → xtrainer.launch.py: 双臂节点 + robot_state_publisher + xtrainer_bridge)
2. `gripper_node` (单节点管理双臂，话题: /gripper/left/xxx, /gripper/right/xxx)
3. `realsense_camera_top` + `realsense_camera_left` + `realsense_camera_right` (关闭 `pointcloud.enable`, 保留 decimation/spatial/temporal/hole_filling/align_depth)
4. `camera_top_point_cloud_xyzrgb` + `camera_left_point_cloud_xyzrgb` + `camera_right_point_cloud_xyzrgb` (depth_image_proc, 消费 aligned_depth_to_color + color + camera_info, 发布 `/camera/<X>/depth/color/points` 在 color 坐标系)
5. 按需加载 easy_handeye2 标定结果 (publish.launch.py)
6. 延迟 3s 后自动使能双臂 (enable_arms)

### 标定启动文件

| 文件 | 相机 | 标定类型 | 标定名 | effector |
| ------ | ------ | ---------- | -------- | ---------- |
| `calibrate_top.launch.py` | camera_top | `eye_on_base` | `top_cam_cal` | L1_6 |
| `calibrate_left.launch.py` | camera_left | `eye_in_hand` | `left_cam_cal` | L1_6 |
| `calibrate_right.launch.py` | camera_right | `eye_in_hand` | `right_cam_cal` | L2_6 |

标定产物路径: `~/.ros2/easy_handeye2/calibrations/<name>.calib`

easy_handeye2 标定参数:

- `calibration_type`: `eye_on_base` 或 `eye_in_hand`
- `name`: 标定唯一名称，对应 .calib 文件名
- `robot_base_frame`: `base_link`
- `robot_effector_frame`: 机械臂末端 link
- `tracking_base_frame`: 相机物理 link (如 `camera_top_link`, `camera_left_link`, `camera_right_link`)
- `tracking_marker_frame`: aruco marker frame (如 `aruco_marker_top`, `aruco_marker_left`, `aruco_marker_right`)

### ArUco 标定板参数

- marker_id: 99
- marker_size: 0.078 (7.8cm)
- corner_refinement: LINES

---

## 已知待配置项

以下配置使用占位符，需要在部署时填入实际值：

- 各相机 `serial_no`: 当前为空字符串 `""`
- 夹爪 `port`: `/dev/ttyUSB0`、`/dev/ttyUSB1`
- 各相机 `camera_frame`: 标定使用物理 link (`camera_top_link`, `camera_left_link`, `camera_right_link`), aruco_ros 的 `reference_frame` 与 `camera_frame` 均设为该 link, easy_handeye2 的 `tracking_base_frame` 也使用该 link (方案A: 标出机械安装关系, 相机内部 TF 由 RealSense 驱动发布)

---

## MoveIt 控制配置

### 架构概览

```
MoveIt (move_group)
  └─ /Arm1_controller/follow_joint_trajectory (FollowJointTrajectory Action)
  └─ /Arm2_controller/follow_joint_trajectory (FollowJointTrajectory Action)
        ↓
xtrainer_controller (双臂 FollowJointTrajectory Action Server, 独立 node)
  └─ 逐点调用 ServoJ 服务 (rad→deg 转换, t=相邻dt, fire-and-forget)
        ↓
dobot_bringup_v4 (cr_robot_ros2_node, 每臂一个, /Arm1 /Arm2 namespace)
  └─ TCP Dashboard (端口 29999) 发送 ServoJ(j1,...,j6,t=...) 命令
  └─ TCP RealTime (端口 30004) 接收关节状态反馈

dobot_bringup_v4 ──► /ArmX/joint_states_robot ──► xtrainer_joint_states (独立 node, 事件驱动合并)
                                                    └─► /joint_states (12 关节)
```

### MoveIt 包 (`moveit_test`)

| 文件 | 说明 |
| ------ | ------ |
| `config/moveit_controllers.yaml` | MoveIt controller manager 配置, 指向 `/ArmX_controller/follow_joint_trajectory` |
| `config/joint_limits.yaml` | 关节速度/加速度限制 (控制 TOTG 时间参数化) |
| `config/pilz_cartesian_limits.yaml` | Pilz 规划器笛卡尔限制 |
| `config/ompl_planning.yaml` | OMPL pipeline + Ruckig 平滑 (覆盖 Jazzy 默认) |
| `config/x_trainer.urdf.xacro` | URDF 入口 (不含 ros2_control, 避免 action server 冲突) |
| `config/x_trainer.srdf` | 规划组定义: Arm1 (base_link→L1_6), Arm2 (base_link→L2_6) |
| `launch/demo.launch.py` | MoveIt demo 启动 (自定义, 不启动 ros2_control_node) |

### 关键配置值

#### joint_limits.yaml (TOTG 时间参数化)

```yaml
default_velocity_scaling_factor: 0.3
default_acceleration_scaling_factor: 0.1
# 每个关节 (J1_1~J1_6, J2_1~J2_6):
has_velocity_limits: true
max_velocity: 3.14          # rad/s, ×0.3 scaling = 0.94 rad/s ≈ 54°/s
has_acceleration_limits: true
max_acceleration: 3.14      # rad/s², ×0.1 scaling = 0.314 (Jazzy TOTG 强制要求)
```

#### ompl_planning.yaml (Ruckig 平滑)

```yaml
# 在 TOTG 之后加 Ruckig, 把 bang-bang 加速度转成 jerk 有限 S 曲线
response_adapters:
  - default_planning_response_adapters/AddTimeOptimalParameterization
  - default_planning_response_adapters/AddRuckigTrajectorySmoothing   # ← 关键
  - default_planning_response_adapters/ValidateSolution
  - default_planning_response_adapters/DisplayMotionPath
```

#### pilz_cartesian_limits.yaml (照官方 Nova2)

```yaml
max_trans_vel: 0.5    # m/s
max_trans_acc: 1.0    # m/s²
max_trans_dec: -2.0   # m/s²
max_rot_vel: 0.785    # rad/s
```

#### URDF 角度限制 (照官方 Nova2 逐关节)

| 关节序号 | 1 | 2 | 3 | 4 | 5 | 6 |
| --------- | --- | --- | --- | --- | --- | --- |
| 角度范围 (rad) | ±6.28 | ±3.14 | ±2.79 | ±6.28 | ±6.28 | ±6.28 |
| (度) | ±360 | ±180 | ±160 | ±360 | ±360 | ±360 |

URDF effort/velocity 全部设为 `0` (=不限制), 清理 SolidWorks 导出垃圾值。关节级限速由 `joint_limits.yaml` 管理。

### xtrainer_bridge 包: 两个独立 node

拆分原因: 单 node 合并 joint_states + 轨迹执行时, ServoJ 的 call_async 在同 executor 内竞争线程, 导致 /joint_states 频率不稳。拆成两个独立进程彻底隔离。

#### `xtrainer_joint_states` (对照官方 dobot_moveit/joint_states.py)

事件驱动合并双臂 joint_states, 无 timer, rclpy.spin 单线程。

| 参数 | 默认值 | 说明 |
| ------ | -------- | ------ |
| `arm1_joint_names` | ['J1_1'..'J1_6'] | 左臂关节名 |
| `arm2_joint_names` | ['J2_1'..'J2_6'] | 右臂关节名 |
| `arm1_joint_state_topic` | /Arm1/joint_states_robot | 左臂状态话题 |
| `arm2_joint_state_topic` | /Arm2/joint_states_robot | 右臂状态话题 |
| `dummy_joint_names` | ['J1_7','J1_8','J2_7','J2_8','J3_1'~'J3_6','J4_1'~'J4_6'] | 哑关节 (无驱动, 发布 0 值消除 MoveIt warning) |

#### `xtrainer_controller` (对照官方 dobot_moveit/action_move_server.py)

双臂 FollowJointTrajectory→ServoJ, MultiThreadedExecutor + ReentrantCallbackGroup。
**同步 execute_callback** (非 async, 避免 time.sleep 阻塞事件循环), `time.monotonic()` 绝对时刻对齐。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm1_servoj_service` | /Arm1/dobot_bringup_ros2/srv/ServoJ | 左臂 ServoJ 服务 |
| `arm2_servoj_service` | /Arm2/dobot_bringup_ros2/srv/ServoJ | 右臂 ServoJ 服务 |

> **注意**: 旧的 `xtrainer_bridge_node` 仍保留但不再使用, 被 `xtrainer_joint_states` + `xtrainer_controller` 替代。

### MoveIt 启动方式

```bash
# 1. 先启动驱动+桥接 (提供 action server + joint_states)
ros2 launch xtrainer_control start.launch.py

# 2. 验证 action server 唯一 (不能有 ros2_control 冲突)
ros2 action list | grep follow_joint_trajectory

# 3. 启动 MoveIt
ros2 launch moveit_test demo.launch.py
```

---

## ServoJ 协议要点

### 官方文档

```
ServoJ(J1,J2,J3,J4,J5,J6,t,aheadtime,gain)
```

- **J1~J6**: 目标关节角度, 单位 **度** (不是弧度!)
- **t** (可选): 该点位的运行时间, 秒, 范围 [0.02, 3600.0], 默认 0.1
- **aheadtime** (可选): 前瞻量, 范围 [20.0, 100.0], 默认 50
- **gain** (可选): 比例增益, 范围 [200.0, 1000.0], 默认 500
- 调用频率建议 33Hz (30ms 间隔)
- 从控制器 3.5.5 起不受全局速度影响

### ROS 服务接口 (`dobot_msgs_v4/srv/ServoJ`)

```
float64 a  # J1 (度)
float64 b  # J2
float64 c  # J3
float64 d  # J4
float64 e  # J5
float64 f  # J6
string[] param_value  # 可选参数, 如 ["t=0.05", "lookahead_time=50", "gain=500"]
---
int32 res
```

### 驱动解析 (`parseTool.cpp`)

`parserServoJRequest2String` 把请求拼成 TCP 字符串:

- 基本形式: `ServoJ(j1,j2,j3,j4,j5,j6)`
- 带参数: `ServoJ(j1,j2,j3,j4,j5,j6,t=0.05)`
- **不做 rad→deg �换**, 调用方必须传度

### 驱动关节状态 (`command.cpp`)

- RealTime TCP (端口 30004) 接收 `q_actual[6]` (度), 控制器每 8ms (125Hz) 推送 1440 字节
- `recvTask` 先写入栈上 `local_data`, 再在 `mutex_` 下拷贝到 `real_time_data_` 和 `current_joint_[]` (deg→rad)
- `getRealData()` 在 `mutex_` 下返回 `RealTimeData` 值拷贝 (不再返回 shared_ptr)
- 因此 MoveIt 看到的是弧度, 发给 ServoJ 时必须转回度

---

## 官方 SDK 参考

### 路径

```
/opt/Project/XTrainer/SDK/DOBOT_6Axis_ROS2_V4/
```

### 关键文件

| 文件 | 说明 |
| ------ | ------ |
| `dobot_moveit/dobot_moveit/action_move_server.py` | 官方 FollowJointTrajectory→ServoJ 桥接 (参考实现) |
| `dobot_moveit/dobot_moveit/joint_states.py` | 官方 joint_states 转发 (单臂 passthrough) |
| `nova2_moveit/config/*.yaml` | 官方 Nova2 MoveIt 配置 (对比基准) |
| `dobot_rviz/urdf/nova2_robot.urdf` | 官方 Nova2 URDF |

### 官方桥接核心逻辑 (`action_move_server.py`)

```python
# 不本地插补, 直发 MoveIt waypoint
for i, (joint, tfs) in enumerate(points_with_time):
    joint = [180 * j / 3.14159 for j in joint]  # rad→deg
    t = tfs - prev_tfs if i > 0 else 0.05       # t=相邻时间差
    t = max(0.004, min(t, 3600.0))
    # 按绝对时刻对齐
    time.sleep(max(0, tfs - elapsed))
    # fire-and-forget
    ServoJ_l.call_async(req)  # param_value=[f"t={t}"]
```

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
| ------ | ------ | ------ |
| `enable_arm(node, ns)` | 单臂使能 (PowerOn→轮询RobotMode→EnableRobot)，已使能则跳过 | node, arm_namespace, timeout=30s |
| `disable_arm(node, ns)` | 单臂去使能 (DisableRobot) | node, arm_namespace, timeout=10s |
| `start_drag(node, ns)` | 单臂拖拽示教开 | node, arm_namespace, timeout=5s |
| `stop_drag(node, ns)` | 单臂拖拽示教关 | node, arm_namespace, timeout=5s |
| `enable_all(node, nss)` | 多臂使能，默认 `["Arm1","Arm2"]` | node, namespaces, timeout=10s |
| `disable_all(node, nss)` | 多臂去使能 | node, namespaces, timeout=10s |

### 控制 launch 文件

| 文件 | 功能 |
| ------ | ------ |
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

---

## 本次会话修改记录 (MoveIt 控制调优, 2026-07-17)

### `moveit_test` — 移除 ros2_control 冲突

- [x] **根因**: `demo.launch.py` 调 `generate_demo_launch` 自动启动 `ros2_control_node`(mock_components/GenericSystem), 与 `xtrainer_bridge` 在同名 `/ArmX_controller/follow_joint_trajectory` 上创建 action server 冲突 → MoveIt 报 `unknown goal response` / `unknown result response`
- [x] `config/x_trainer.urdf.xacro`: 移除 `<xacro:include x_trainer.ros2_control.xacro>` 及 `<ros2_control>` macro 调用
- [x] `launch/demo.launch.py`: 改为手写版, 不启动 `ros2_control_node` 与 `spawn_controllers`, 用 `DeclareBooleanLaunchArg` 代替不存在的 `DeclareBooleanLaunchAction`

### `xtrainer_bridge` — ServoJ 单位修复 (rad→deg)

- [x] **根因**: Dobot 协议 ServoJ 的 J1~J6 单位是**度**, 但 MoveIt 规划是弧度, bridge 直接转发导致控制器把 0.138rad 当成 0.138° 处理 → "微微动一下就不动"
- [x] `_send_servoj` 新增 `degrees()` 转换
- [x] 新增参数 `servoj_angle_unit` (默认 'rad')

### `xtrainer_bridge` — 重写轨迹执行 (照官方 action_move_server.py)

- [x] **根因**: 本地线性插补在 waypoint 边界产生速度阶跃 → 加加速度突变 → 卡顿/刹车感; 且 fire-and-forget 无背压导致队列堆积
- [x] 删除 33Hz 重采样与线性插补逻辑
- [x] 直发 MoveIt waypoint, `t = 相邻 time_from_start 差值` (首点 `t=0.05`)
- [x] 绝对时刻节拍对齐 (`time.sleep(cur_tfs - elapsed)`)
- [x] fire-and-forget `call_async` (不等 driver TCP echo)
- [x] 新增参数 `joint_state_publish_rate` (50Hz, 与 driver 同频, 解耦于轨迹执行)
- [x] 保留 rad→deg 转换、MultiThreadedExecutor + ReentrantCallbackGroup (双臂必需)

### `cr_robot_ros2` (driver) — 修复 TCP 忙循环

- [x] **根因**: `command.cpp` 的 `doTcpCmd` 中 `tcpRecv(timeout=0)` 非阻塞 poll + `sleep(0.01)` 被截断为 `sleep(0)` → CPU 100% 忙轮询
- [x] `tcpRecv` timeout 改为 100ms (select 阻塞, 数据到达立即返回)
- [x] `sleep(0.01)` 改为 `std::this_thread::sleep_for(std::chrono::milliseconds(1))`
- [x] 影响 `doTcpCmd` 和 `doTcpCmd_f` 两处

### `moveit_test` — joint_limits / pilz / URDF 限制对齐官方

- [x] **根因**: URDF effort/velocity 是 SolidWorks 导出的 float32 最大值垃圾, MoveIt TOTG 读取后生成激进 bang-bang 时间参数 → 短 t → 控制器来不及平滑 → 卡顿
- [x] `joint_limits.yaml`: `has_velocity_limits: true, max_velocity: 3.14, has_acceleration_limits: true, max_acceleration: 3.14` (让 TOTG 生成平滑梯形)
- [x] `pilz_cartesian_limits.yaml`: 照官方降低 (trans_vel 1.0→0.5, trans_acc 2.25→1.0, trans_dec -5.0→-2.0, rot_vel 1.57→0.785)
- [x] `x_trainer.urdf`: 角度限制照官方逐关节 (J1/J4/J5/J6=±6.28, J2=±3.14, J3=±2.79), effort/velocity 全清零

### `moveit_test` — 新增 Ruckig 平滑 + 调整 scaling

- [x] **根因**: TOTG 生成 bang-bang 加速度 (瞬时 ±max), 加加速度(jerk)无穷大 → 控制器内部插补器追不上 → 卡顿
- [x] 新建 `config/ompl_planning.yaml`: 在 TOTG 后加 `AddRuckigTrajectorySmoothing`, 把 bang-bang 转 jerk 有限 S 曲线
- [x] `joint_limits.yaml` scaling 调整: velocity 0.1→0.3 (提速), acceleration 保持 0.1 (低加速度更平滑)

### `cr_robot_ros2` (driver) — MultiThreadedExecutor + 回调组隔离

- [x] **根因**: `main.cpp` 用 `while`+`spin_some` 单线程循环, ServoJ 回调阻塞 TCP echo ~10-15ms 卡住 joint_states 发布 → /joint_states_robot 不稳 → bridge /joint_states 跟着不稳
- [x] `main.cpp`: `while`+`spin_some` → `MultiThreadedExecutor` + `wall_timer` 发布 joint_states
- [x] `cr_robot_ros2.h`: 新增 `servo_cb_group_` 成员
- [x] `cr_robot_ros2.cpp`: ServoJ/ServoP 服务放入独立 `MutuallyExclusive` callback group, 与 timer 发布隔离

### `xtrainer_bridge` — 拆分为两个独立 node (终极修复)

- [x] **根因**: 单 `xtrainer_bridge_node` 内 joint_states timer 与轨迹执行的 ServoJ call_async 在同一 executor 内竞争线程, 即使 MultiThreadedExecutor(8线程) + callback group 隔离仍不稳
- [x] 新增 `xtrainer_joint_states.py` — 事件驱动合并双臂 joint_states (照官方 joint_states.py)
- [x] 新增 `xtrainer_controller.py` — FollowJointTrajectory→ServoJ (照官方 action_move_server.py, async def execute_callback)
- [x] `setup.py`: 新增两个入口点 `xtrainer_joint_states` 和 `xtrainer_controller`
- [x] `dobot_bringup_v4/launch/xtrainer.launch.py`: 单 bridge_node → 两个独立 node

---

## MoveIt 调优经验教训

### 排错决策树 (运动不正常时)

1. **完全不动**: 检查 action server 是否冲突 (`ros2 action list`), 检查 ServoJ 单位 (rad→deg)
2. **微微动一下**: 99% 是 rad→deg 单位没转 (0.138rad 被当成 0.138°)
3. **能动但卡顿**: 检查轨迹执行策略 (是否本地插补), 检查 t 参数 (是否固定值 vs 相邻dt), 检查 driver TCP 阻塞
4. **连续运动中刹车**: 检查 t 是否远大于发送间隔 (末端减速), 检查 joint_limits (TOTG bang-bang)
5. **MoveIt execute 失败/timeout**: 检查是否有 ros2_control 与 bridge 的 action server 冲突
6. **/joint_states 频率不稳**: 检查是否单 node 内 joint_states 与 ServoJ 竞争线程 → 拆成独立 node

### 关键陷阱

| 陷阱 | 后果 | 对策 |
| ------ | ------ | ------ |
| ros2_control + bridge 同名 action server | goal/result 路由错乱 | 移除 ros2_control 或用不同 action 名 |
| ServoJ 传弧度不传度 | 几乎不动 | bridge 做 `degrees()` 转换 |
| 本地线性插补 | waypoint 边界速度阶跃 → 卡顿 | 直发 waypoint, 依赖控制器内部插补 |
| t=固定值 ≠ 相邻dt | 末端刹车或跟踪不一致 | t=相邻 time_from_start 差值 |
| URDF velocity/effort 垃圾值 | TOTG 生成激进 bang-bang | URDF 设 0, joint_limits.yaml 管限速 |
| has_acceleration_limits=false | **Jazzy TOTG 直接拒绝运行**(报错) | 必须 true + 合理 max_acceleration |
| TOTG 无 Ruckig 平滑 | bang-bang 加速度 → jerk 无穷大 → 卡顿 | 加 `AddRuckigTrajectorySmoothing` |
| call_async 无背压 | driver 队列堆积 → 突发下发 → 抖动 | 绝对时刻节拍对齐 (但 fire-and-forget 即可) |
| joint_states 频率 < driver 频率 | /tf 延迟/跳变 | joint_state_publish_rate=50, 与 driver 同频 |
| driver `sleep(0.01)` = `sleep(0)` | CPU 100% 忙轮询 | `sleep_for(1ms)` + `tcpRecv(timeout=100)` |
| driver 单线程 spin_some + ServoJ 阻塞 | /joint_states_robot 跌到 30Hz 且不稳 | MultiThreadedExecutor + 独立 callback group |
| 单 node 内 joint_states + ServoJ 同 executor | 线程竞争 → /joint_states 不稳 | 拆成两个独立 node (进程隔离) |
| `DeclareBooleanLaunchAction` | ImportError (不存在) | 用 `moveit_configs_utils.launch_utils.DeclareBooleanLaunchArg` |
| driver `doTcpCmd` 每次 3 条 `std::cout` | 33Hz×双臂=198 条 stdout/s, `endl` flush → I/O 抖动 | `#define DEBUG 0` 关闭 |
| driver `wall_timer` 与 ~70 服务共享默认 group | 非 ServoJ 服务调用阻塞 → joint_states 发布暂停 | timer 独立 callback group |
| `dash_board_tcp_` 无互斥锁 | servo_cb_group_ 与默认 group 并发写同一 socket → 命令交错损坏 | `tcp_mutex_` 保护 doTcpCmd |
| `recvTask` 直接写 `real_time_data_` | `pubFeedBackInfo` 100Hz 无锁读 → data race | 先写栈 local buffer, 再 mutex 拷贝 |
| `xtrainer_controller` async def + time.sleep | 阻塞 asyncio 事件循环 → 双臂并发受限 | 改为同步 def, MultiThreadedExecutor 分配独立线程 |

---

## 本次会话修改记录 (夹爪重构, 2026-07-20)

### `xtrainer_gripper` — 双臂单节点架构

- [x] **根因**: 原有两个独立节点 (每个夹爪一个进程) 冗余且不便统一管理
- [x] 重构 `gripper_node.py`: 单 `GripperNode` 管理 `_GripperDriver` ×2 (左/右), 各自独立串口和舵机 ID
- [x] 命名空间 `/gripper`, 话题路径 `/gripper/left/xxx`, `/gripper/right/xxx`
- [x] 左夹爪: `/dev/ttyUSB0`, 舵机 ID=21；右夹爪: `/dev/ttyUSB1`, 舵机 ID=22
- [x] 配置: `left_*` / `right_*` 分组参数, 各侧独立 `servo_min_pos`/`servo_max_pos`

### `xtrainer_gripper` — 新增话题

- [x] `~/left/position`, `~/right/position` (Int32) — 电机原始位置读数 (0~4095)
- [x] `~/left/torque`, `~/right/torque` (Int32) — 运行时设定力矩限制 (0~1000), 订阅后立即写入舵机
- [x] `~/left/load_raw`, `~/right/load_raw` (Int32) — 原始负载读数 (0~1000)

### `xtrainer_gripper` — 开/关方向修正

- [x] **根因**: `_servo_min`(角度下限)=闭合, `_servo_max`(角度上限)=张开, 但映射写反了
- [x] `_normalized_to_servo`: 0.0→`_servo_max`(张开), 1.0→`_servo_min`(闭合)
- [x] `_servo_to_normalized`: 读到大概率→接近 0.0(OPENED), 读到小值→接近 1.0(CLOSED)

### `xtrainer_gripper` — 移除 sudo 依赖

- [x] `_set_latency_timer()`: 改为直接写入 sysfs, PermissionError 静默跳过
- [x] 新增 `setup_udev.sh`: 一键安装 udev 规则 `SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"`

### `xtrainer_gripper` — 新增控制类与一键脚本

- [x] 新增 `gripper_control.py`: `GripperController` 类, 通过 topic 控制夹爪
  - `open(side)` / `close(side)` / `open_both()` / `close_both()` — 运动控制
  - `set_torque(side, limit)` / `set_torque_both(limit)` — 力矩控制
  - `read_position(side)` / `read_load(side)` / `read_raw_position(side)` / `read_raw_load(side)` / `read_status(side)` — 非阻塞读取
  - `wait_status(side, target, timeout)` — 阻塞等待到位
- [x] 新增 `gripper_open_arms.py` / `gripper_close_arms.py` — 一键开关脚本
- [x] 新增 `launch/gripper_open.launch.py` / `launch/gripper_close.launch.py`

### `xtrainer_gripper` — 其他

- [x] 行程限位改为 launch 配置文件方式, 不再从舵机 EPROM 读取
- [x] 更新 `start.launch.py`: 合并两个独立 Node 为单一 `gripper_node`
- [x] 更新 `gripper.launch.py`: 适配 `left_*` / `right_*` 参数结构与环境变量
- [x] 更新 `setup.py`: 注册新入口点和 launch 文件
- [x] 更新 Demo 脚本: 新增 `gripper_side` 参数适配新话题结构
- [x] 更新 `README.md`: 反映双臂单节点架构和 GripperController API

### 夹爪话题完整清单

| 话题 | 类型 | 方向 | 说明 |
| ------ | ------ | ------ | ------ |
| `/gripper/left/command` | Float32MultiArray | 订阅 | `[position(0-1), speed(0-1)]` |
| `/gripper/left/state` | Float32MultiArray | 发布 | `[position(0-1), load(0-1)]` |
| `/gripper/left/status` | String | 发布 | OPENED/CLOSED/MOVING/ERROR |
| `/gripper/left/position` | Int32 | 发布 | 原始位置 0~4095 |
| `/gripper/left/torque` | Int32 | 订阅 | 运行时力矩限制 0~1000 |
| `/gripper/left/load_raw` | Int32 | 发布 | 原始负载 0~1000 |
| `/gripper/right/*` | (同上) | | |

---

## 本次会话修改记录 (xtrainer_task MoveItPy 可视化, 2026-07-20)

### MoveItPy 架构说明

MoveItPy 是**进程内**规划库，与 move_group 是**平级替代**，不是 client：

| 架构 | 进程 | 需要自己的 config | RViz 支持 |
| ------ | ------ | ------ | ------ |
| move_group + MoveGroupInterface | 独立进程，暴露 MoveGroup Action | move_group 自己加载 | 完整 MotionPlanning 交互面板 |
| MoveItPy + PlanningComponent | 进程内库 | 需要 moveit_cpp.yaml + MoveItConfigsBuilder | PlanningSceneDisplay + 手动轨迹发布 |

> **注意**: ROS2 Jazzy 的 `moveit_py` 没有提供 `MoveGroupInterface` 的 Python 绑定（只有 C++ 头文件）。如需从 Python 连接 move_group，需用 C++ + pybind11（参考 `/opt/Project/ros_dual_arm/src/start_controller/`）。

### MoveItPy 与 move_group 共存

可以同时运行，**只要不同时执行轨迹**：

- PlanningSceneMonitor 各用不同 topic namespace（`/move_group/` vs `/moveit_cpp/`），不冲突
- /joint_states 多订阅者正常
- **唯一冲突点**: 两者的 `TrajectoryExecutionManager` 可能同时向 `/ArmX_controller/follow_joint_trajectory` 发 goal

### `xtrainer_task/config/moveit_cpp.yaml` — 重写

- [x] **根因**: 旧版 `planning_pipelines: {pipeline_names: [...]}` 覆盖了 `MoveItConfigsBuilder.planning_pipelines()` 自动加载的 pipeline 配置（丢失每个 pipeline 的 `planning_plugins` 等参数），导致 MoveItCpp 报 `Failed to load any planning pipelines`
- [x] `MoveItConfigsBuilder` 生成的 `planning_pipelines` 是 flat list `["ompl", ...]`，但 MoveItCpp 期望 nested dict `{pipeline_names: ["ompl", ...]}`
- [x] 重写为只含 `planning_scene_monitor_options`、`plan_request_params`、`ompl_rrtc`、`pilz_ptp`，不再覆盖 `planning_pipelines` dict

### `moveit_test/config/sensors_3d.yaml` — 禁用 octomap

- [x] 旧版引用不存在的 kinect topic，导致 `No 3D sensor plugin(s) defined for octomap updates` 错误
- [x] 改为 `sensors: []`

### `xtrainer_task/config/xtrainer.rviz` — 新增

- [x] 使用 `moveit_rviz_plugin/PlanningScene` 订阅 `/moveit_cpp/monitored_planning_scene` 显示机器人模型
- [x] 使用 `moveit_rviz_plugin/Trajectory` 订阅 `/display_planned_path` 显示规划轨迹
- [x] Fixed Frame: `base_link`
- [x] 注意：这是 `PlanningSceneDisplay`，不是 `MotionPlanningDisplay`（后者需要 move_group）

### `xtrainer_task/launch/start.launch.py` — 增加 RViz

- [x] 新增 `use_rviz` launch arg（默认 true）
- [x] 新增 RViz 节点，传入 `robot_description` + `robot_description_semantic` 参数
- [x] `use_rviz:=false` 可禁用

### `xtrainer_task/robot_move.py` — 增加轨迹可视化

- [x] 新增 `_display_pub` 发布 `/display_planned_path` (DisplayTrajectory)
- [x] 新增 `_display_trajectory()` 方法：规划成功后自动推送轨迹到 RViz 显示
- [x] `plan_joints()` / `plan_pose()` 规划成功后自动调用 `_display_trajectory()`
- [x] `get_current_pose()` / `plan_pose()` / `plan_and_execute_pose()` 支持可选
  `tip_link` 参数；省略时默认使用 `L1_gripper_tcp` / `L2_gripper_tcp`
- [x] 新增 tip link 可显式选择，例如：
  `mover.plan_pose('Arm1', pose, tip_link='L1_gripper_tip')`

### `xtrainer_task/setup.py` — 注册新文件

- [x] 注册 `config/xtrainer.rviz`

---

## 本次会话修改记录 (2026-07-22)

### `xtrainer_task/xtrainer_task/start.py` — 修复 float 索引 + 新增检测点 RViz Marker

- [x] **根因**: `launch()` 中 `mid_x`/`mid_y` 为 float, 传入 `get_depth_at_pixel` 后切片索引变成 float → `TypeError`
- [x] `get_depth_at_pixel` 开头新增 `u=int(u); v=int(v)` 强制类型转换
- [x] 新增 `_marker_pub` (topic `/detection_marker`) + `_publish_detection_marker()` 方法, 在 `pixel_to_base_link` 成功后发布红色小球 Marker 到 RViz
- [x] **QoS 踩坑**:
  - 初版用 `~/detection_marker` → 但 launch 文件覆写 node_name 为 `xtrainer_task_moveit`, topic 错位
  - 改用绝对 topic `/detection_marker`
  - Mark 只在启动 1s 后发布一次, RViz 加载需 3-5s → 默认 VOLATILE QoS 丢消息
  - 改为 `TRANSIENT_LOCAL` durability 解决
- [x] 新增 `mid_coordinate is None` 空值检查, 避免 crash

### `xtrainer_task/config/xtrainer.rviz` — 新增 DetectionMarker display

- [x] 在 Trajectory display 后新增 `rviz_default_plugins/Marker`, 订阅 `/detection_marker`, namespace `detection`

### `xtrainer_bridge/xtrainer_bridge/xtrainer_joint_states.py` — 新增哑关节发布

- [x] **根因**: URDF 定义了 J1_7/J1_8 (夹爪手指), J3_1~J3_6 (Arm3), J4_1~J4_6 (Arm4) 共 16 个无驱动关节, MoveIt PlanningSceneMonitor 持续报 `Missing JX_X` warning
- [x] 新增 `dummy_joint_names` 参数 (可通过 launch 覆写), 默认包含全部 16 个哑关节
- [x] `_publish()` 时将哑关节 (全 0 值) 追加到 `/joint_states` 消息, 消除 MoveIt 警告
- [x] 更新参数表: 新增 `dummy_joint_names` 行

---

## 本次会话修改记录 (2026-07-27)

### `xtrainer_description` — 新增抓取尖端 frame

- [x] 新增 `L1_gripper_tip` / `J1_gripper_tip` fixed joint
- [x] 新增 `L2_gripper_tip` / `J2_gripper_tip` fixed joint
- [x] 两个 tip 当前均位于对应 `L*_6` 的局部 Z 轴正方向，距离为
  `0.195 m` (19.5 cm；以 URDF 当前实际值为准)
- [x] 原有 `L1_gripper_tcp` / `L2_gripper_tcp` 保留，默认规划行为不变

### `xtrainer_task` — tip link 选择与吸管抓取

- [x] `RobotMover.get_current_pose()`、`plan_pose()` 和
  `plan_and_execute_pose()` 新增可选 `tip_link` 关键字参数
- [x] 省略 `tip_link` 时继续使用 TCP，确保既有任务兼容
- [x] `start_insert_straw.py` Step2 使用 `L1_gripper_tip` 进行抓取点位姿
  查询和下降规划
- [x] Step2 使用 DINO/SAM2 mask 主轴 + 深度 + TF 估计吸管三维方向，
  仅旋转 `J1_6` 后执行抓取

---

## 本次会话修改记录 (夹爪命令可靠性, 2026-08-02)

### 问题现象

- 现象1: 快速连发开/合命令时, 部分命令不执行
- 现象2: 通过 CLI 脚本 (`gripper_open_arms`/`gripper_close_arms`) 触发时偶发"命令收不到" (`gripper_node` 日志里没有 `[left/right] 接收到命令`)

### `xtrainer_gripper/xtrainer_gripper/gripper_node.py` — 命令覆盖丢失 → FIFO 队列

- [x] **根因**: 命令处理用"最新覆盖"模式 (`_latest_cmd` 单变量) + 30ms 处理定时器. 两条命令间隔 < 30ms 时, 前一条在执行前被覆盖, 永远不会下发到舵机
- [x] `_latest_cmd` → `queue.Queue()`, FIFO 不丢命令
- [x] `_cmd_callback`: `queue.put()`; `_process_command`: `queue.get_nowait()` (仍 30ms 定时器每次取 1 条, 串口写速率上限不变 ≤33Hz, 不增加发送频率)
- [x] 佐证: `gripper_open_arms.py` 早有 `command_repeats=3` / `command_interval=0.1` 重发补丁在绕这个 bug

### `xtrainer_gripper/xtrainer_gripper/gripper_node.py` — 清理重复串口读

- [x] **问题**: `_publish_state` 位置/负载各读两次串口 (raw 一次 + normalized 又一次), 20Hz 下浪费 ~6ms/周期 RS485 总线, 挤占单线程执行器
- [x] 合并为各读一次 raw, normalized 值本地推导 (`_servo_to_normalized` + `raw/1000`)
- [x] 删除冗余 `_read_position`/`_read_load`, 在 `_read_raw_*` 中补回 throttled debug 日志

### `xtrainer_gripper/xtrainer_gripper/gripper_control.py` — DDS 发现丢消息 → 发送前等订阅匹配

- [x] **根因**: CLI 脚本是短生命周期节点, 创建 publisher 后 publish 时若 DDS pub↔sub 匹配尚未完成, VOLATILE QoS 下该条直接丢弃 (无 late-joining). `gripper_close_arms` 默认 `command_repeats=1` (只发一次), 比 `open` (`repeats=3`) 更易丢
- [x] 新增 `_ensure_subscriber(side)`: 用 `publisher.get_subscription_count()` 确认已匹配, 未匹配则有界等待 (默认 2s); 匹配后即时返回零开销 (一次本地 count 查询, 非 DDS 往返)
- [x] 新增 `_spin_a_bit()`: 兼容两种调用方
  - CLI (节点未挂 executor): `rclpy.spin_once` 推进发现
  - 任务节点 (节点被 `MultiThreadedExecutor` 接管, 如 `start_insert_straw.py`): `spin_once` 抛异常 → `try/except` 退化为 `sleep`, 发现由外部 executor 推进
- [x] `GripperController.__init__` 新增 `wait_timeout` 参数 (默认 2.0)
- [x] 阻塞发生在调用方线程, 不影响 `gripper_node` 进程

### 夹爪可靠性陷阱

| 陷阱 | 后果 | 对策 |
| ------ | ------ | ------ |
| `_latest_cmd` 覆盖 + 30ms 处理定时器 | 间隔 <30ms 的命令被静默丢弃 | 用 FIFO `queue.Queue` |
| `_publish_state` 重复串口读 | RS485 总线占用翻倍, 挤占执行器线程 | raw 只读一次, normalized 本地推导 |
| 短生命周期节点 publish 早于 DDS 匹配 | VOLATILE QoS 丢首条命令 (偶发"收不到") | 发送前 `get_subscription_count()` 等匹配 |
| `gripper_close_arms` `repeats=1` | 单发容错为 0, 最易受发现丢失影响 | 现由 `_ensure_subscriber` 保证; `repeats` 补丁已非必需 (保留作冗余) |

### 说明 (本次未改动)

- `gripper_node` 仍为 `rclpy.spin` 单线程执行器: 状态发布做同步阻塞串口读, 偶发"收到但延迟"是潜在问题, 但**不是**本次"收不到"的原因 (本次是 DDS 发现层丢失). 若后续出现响应延迟, 再上 `MultiThreadedExecutor` + callback group 隔离 (同 dobot driver / xtrainer_bridge 的拆分经验)

---

## 本次会话修改记录 (驱动通信可靠性 + bridge 重构, 2026-08-02)

### 官方 TCP/IP 协议文档确认

协议文档: `/opt/Project/TCP-IP-Protocol-6AXis-V4/Dobot TCP_IP二次开发接口文档V4.5.1_20240815_cn.md`

- **端口 29999 (Dashboard)**: ASCII 文本, 请求-响应, 所有控制指令 (ServoJ/MovJ/EnableRobot…)
- **端口 30004 (RealTime)**: 二进制 1440 字节, 控制器每 **8ms (125Hz)** 推送 (QActual/ToolVectorActual/RobotMode 等)
- 端口 30005 (200ms)、30006 (默认 1000ms, 可配) — driver 未使用
- 两条连接均为**长连接** (persistent), 断线自动重连

### `dobot_bringup_v4` — driver 三重隔离 + 通信安全

- [x] **根因 1**: `wall_timer` 与 ~70 个非 ServoJ 服务共享默认 callback group → 任何服务调用阻塞 TCP echo 时 joint_states 发布被排队
- [x] `main.cpp`: 新建 `timer_cb_group` (MutuallyExclusive), wall_timer 挂载其上, 与 servo_cb_group_ 和默认 group 三方隔离
- [x] **根因 2**: `dash_board_tcp_` socket 无互斥锁 → servo_cb_group_ 与默认 group 的服务并发写同一 socket, TCP 命令交错损坏
- [x] `command.h`: 新增 `tcp_mutex_` 成员; `doTcpCmd`/`doTcpCmd_f` 从 `static` 改为实例方法; `mutex_` 改为 `mutable`
- [x] `command.cpp`: `doTcpCmd`/`doTcpCmd_f` 开头加 `std::lock_guard<std::mutex> tcp_lock(tcp_mutex_)`
- [x] **根因 3**: `doTcpCmd` 每次 3 条 `std::cout << ... << std::endl` (发送时间+命令、ErrorID、完整响应), 33Hz×双臂 = 198 条 stdout/s
- [x] `command.cpp`: 新增 `#define DEBUG 0`, 3 条日志全部 `#if DEBUG` 包裹 (硬编码关闭, 与 CMake 无关; 开发时改 `1` 即恢复)
- [x] **根因 4**: `recvTask` 直接写 `real_time_data_` (shared_ptr 指向的缓冲区), `pubFeedBackInfo` 100Hz 无锁读同一缓冲区 → data race
- [x] `command.cpp` `recvTask`: `tcpRecv` 改为写入栈上 `RealTimeData local_data`, 验证 `len==1440` 后在 `mutex_` 下 `memcpy` 到 `real_time_data_`
- [x] `command.h`/`command.cpp`: `getRealData()` 返回类型 `shared_ptr<RealTimeData>` → `RealTimeData` (值拷贝), 在 `mutex_` 下返回
- [x] `cr_robot_ros2.cpp` `pubFeedBackInfo`: `shared_ptr<RealTimeData>` → `RealTimeData`, 所有 `->` → `.`

### `xtrainer_bridge` — controller async→sync 回归修复

- [x] **根因**: `xtrainer_controller.py` 用 `async def execute_callback` 调用同步阻塞函数 (`time.sleep`), 阻塞 asyncio 事件循环; 旧的 `xtrainer_bridge_node.py` 用的是正确的同步 `def`
- [x] `async def` → `def` (3 个方法: `_arm1_execute_cb`, `_arm2_execute_cb`, `_execute`)
- [x] 删除执行前逐点日志 (`_execution_trajectory` 中对每个 waypoint 打 `INFO` 日志, 200 点 = 200 条启动延迟)
- [x] 合并 `_execution_trajectory` + `_execute_trajectory` 为一个函数, 直接遍历原始 `trajectory.points`
- [x] `time.time()` → `time.monotonic()` (单调时钟, 不受 NTP 影响)
- [x] 新增取消检查 (`goal_handle.is_cancel_requested`), service 可用性检查 (`wait_for_service`), 正确的 `succeed`/`canceled`/`abort` 结果处理

---

## 本次会话修改记录 (点云坐标系修复 + depth_image_proc, 2026-08-04)

### `xtrainer_control/launch/start.launch.py` — realsense 点云 → depth_image_proc

- [x] **根因**: `realsense2_camera` 4.58.2 的 `/camera/<X>/depth/color/points` 在 **depth 坐标系** (frame_id = `camera_X_depth_optical_frame`, H×W = decimation 后的 depth 分辨率), SAM mask 在 color 坐标系, 直接像素对齐错位 → `graspnet_sam_test.py`/`start_grasp_garbage.py` 把 mask resize 后取的点云来自物体旁边的区域
- [x] **关键陷阱 — 文档过期**: `realsense-ros/README.md:819-820` 写 "The pointcloud, if created, will be based on the aligned depth image" 是**旧文案**, 描述的是 2023-06 PR #2775 (commit `f4c4ff88`) **之前**的行为。该 PR 把 `_align_depth_filter` push 到 `_pc_filter` **后面** (见 `base_realsense_node.cpp:259-263` 的注释 `// Apply PointCloud filter before applying Align-depth as it requires original depth image not aligned-depth image.`), 之后点云改用原始 depth 生成, 但文档没更新
- [x] 三处 `"pointcloud.enable": "true"` → `"false"` (top/left/right)
- [x] 保留 `decimation/spatial/temporal/hole_filling/align_depth` 不变 —— `/aligned_depth_to_color/image_raw` 仍然带全套滤波 (filter 链在 align_depth 之前), 这就是"官方点云质量"的源头
- [x] 新增 `_make_pc_node(camera_name)` helper + 3 个 `depth_image_proc/point_cloud_xyzrgb_node` 实例
- [x] 每个节点 remap: `depth_registered/image_rect` → `aligned_depth_to_color/image_raw`, `rgb/image_rect_color` → `color/image_raw`, `rgb/camera_info` → `color/camera_info`, `points` → `depth/color/points` (与 realsense 原路径一致)
- [x] depth_image_proc 用 `PointCloud2Modifier.setPointCloud2FieldsByString(2, "xyz", "rgb")` —— 字段结构与 realsense 一致 (x/y/z/rgb FLOAT32, point_step=16), 下游 `_ros_pointcloud_to_organized` 解析逻辑不变
- [x] **结果**: 新点云 frame_id = `camera_X_color_optical_frame`, H×W = color 分辨率 (1280×720), SAM mask 1:1 直接作用

### `xtrainer_task/start_grasp_garbage.py` — TF buffer cache_time 10s → 120s

- [x] **根因**: `tf2_ros.Buffer()` 默认 `cache_time=10s`, Step2 单轮 SAM2(~5-10s) + GraspNet(~5s) 累计已超 10s; 若 TF 失败进入 `while` 重试, 累积达数十秒。点云采集时刻 (`cloud_msg.header.stamp`) 的 TF 早已被丢弃 → `Lookup would require extrapolation into the past. Requested time X but the earliest data is at time Y`
- [x] 与 depth_image_proc 无关: depth_image_proc 在 `point_cloud_xyzrgb.cpp` 里 `cloud_msg->header = depth_msg->header;` 保留 depth 原始时间戳, 新旧流程 stamp 行为一致
- [x] `self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=120))`
- [x] 机械臂 Step2 开始时已静止 (Step1 + `time.sleep(0.5)`), 用 cloud stamp 语义正确, 调大 cache 即可

### `xtrainer_task/{start_grasp_garbage,graspnet_sam_test}.py` — `_dilate_mask_3d` 性能优化 (28×)

- [x] **根因**: 旧 `_dilate_mask_3d` 对**整张点云所有 valid 点**调 `cKDTree.query(k=1)` (`idx = np.where(flat_valid)[0]; tree.query(grid[idx], k=1)`), 复杂度 O(N_valid × log N_mask)。换 depth_image_proc 后点云从 640×360 (~23w 点) 升到 1280×720 (~92w 点), 单次从 ~1s 飙到 2-5s, 是 Step2 "Stub7 之后卡住"的真正元凶
- [x] 新算法: 先 `cv2.dilate` 在 2D 图像域把 mask 扩 N 像素作为候选区 (N 按 `radius_m / median_depth × focal_guess` 自适应, `focal_guess = max(W,H)*0.5`, cap [3, 50]), 再用 cKDTree **仅对候选点** 做精确 3D 距离检查
- [x] benchmark (1280×720, mask 28575 点, radius=4cm, depth=0.7m): 旧 2462ms → 新 87ms, **加速 28×**, `np.array_equal(旧, 新) == True` (结果完全一致, kernel_size=77, candidates=37848)
- [x] 同步改 `graspnet_sam_test.py` 保持两份代码一致
- [x] 清理 10 个 `Stub0..9` 调试打印

### 调试记录: `graspnet_sam_test.py` 误判 c7c4ff44 是 bug

- [x] **现象**: 用户跑 `graspnet_sam_test` 看到 `dets=1 pts=0 grasps=0`, 怀疑 commit `c7c4ff443a0217657014c8e54c998520ded6405a` 改了 dino_wrapper API
- [x] **真相 1**: c7c4ff44 只改了 `scores` 的多维兜底 (multimask_output=False 时 `scores_arr.reshape(-1)`), 完全没动 `masks` 处理; 而 `graspnet_sam_test.py` 只用 `det.boxes` 和 `det.masks` (line 262/269/272), 根本不读 `det.scores` → 与 c7c4ff44 无关
- [x] **真相 2**: 命令笔误 — `pointcloud_topic:=/camera/camera_right/...` + `image_topic:=/camera/camera_top/...` 用了**不同相机**。SAM 在 camera_top 检出 bottle, mask 应用到 camera_right 点云 (看不到 bottle) → `sel = mask2d & valid` 全 False → pts=0
- [x] **修复**: 同一相机 (top 或 right 二选一), 或不指定参数走默认 (camera_top)

### depth_image_proc 关键事实备忘

| 项 | 值 |
| --- | --- |
| package | `depth_image_proc` (ROS2 Jazzy 5.0.12, 已安装) |
| executable | `point_cloud_xyzrgb_node` |
| 默认订阅 | `depth_registered/image_rect`, `rgb/image_rect_color`, `rgb/camera_info` (扫二进制确认) |
| 默认发布 | `points` |
| 同步策略 | `ApproximateTime` (默认) / `ExactTime` (`use_exact_sync:=true`) |
| 深度单位 | uint16 mm (DepthTraits<uint16_t>::toMeters = ×0.001) |
| 输出 frame_id | 取自 depth_msg->header (即 aligned_depth_to_color 的 `camera_X_color_optical_frame`) |
| 字段 | `setPointCloud2FieldsByString(2, "xyz", "rgb")` = x/y/z/rgb FLOAT32, point_step=16 (与 realsense 一致) |
| 输出 ordered | 是, H×W = depth_msg 维度 = color 分辨率 (因 align_depth) |

### 性能特征 (3 × 30 FPS 独立 Node 估算)

- 输入: 每相机 4.6 MB/帧 (color 2.76MB + aligned_depth 1.84MB), 30fps = 138 MB/s
- 输出: PointCloud2 14.7 MB/帧, 30fps = 441 MB/s
- 反投影本身单核 ~5-10% CPU; 主要开销在 PointCloud2 序列化
- 3 × 30 FPS 独立 Node 估算总体 CPU 30-50%, 不是瓶颈
- 真正瓶颈是下游 Python 订阅者 (graspnet_sam_test 单进程消化 30Hz × 14.7MB 会吃满 1 核)
- ComposableNode 对本场景收益主要在内存, 不在 CPU (输出给外部进程仍走 DDS)

### 点云坐标系统一约定 (本次后)

| Topic | frame_id | H×W | 像素 (u,v) 语义 |
| --- | --- | --- | --- |
| `/camera/<X>/color/image_raw` | `camera_X_color_optical_frame` | 1280×720 | color 视角 |
| `/camera/<X>/aligned_depth_to_color/image_raw` | `camera_X_color_optical_frame` | 1280×720 | color 视角 (depth 已 warp) |
| `/camera/<X>/depth/color/points` (新, depth_image_proc) | `camera_X_color_optical_frame` | 720×1280 | color 视角, 与 SAM mask 1:1 |
| ~~`/camera/<X>/depth/color/points` (旧, realsense)~~ | ~~`camera_X_depth_optical_frame`~~ | ~~360×640~~ | ~~depth 视角, 与 SAM mask 错位~~ |

### 排错决策树补充 (SAM 点云分割类)

1. **`pts=0` 但 `dets>0`**: 检查 `image_topic` 和 `pointcloud_topic` 是否同一相机 (`ros2 topic info` 看 publisher); 检查 frame_id 是否一致
2. **`pts` 非零但与物体位置错位**: 点云 frame_id 是 `depth_optical_frame` 还是 `color_optical_frame`? 旧 realsense 自带点云用前者, 必错位
3. **SAM mask 在 OpenCV 窗口正确但点云跟不上**: 检查点云 H×W 是否等于 color 分辨率; 检查 `_select_mask` 里 resize 是否触发 (触发即说明点云网格不在 color 域)
4. **Step2 卡数十秒后报 TF extrapolation into the past**: TF buffer cache_time 10s < SAM+GraspNet 推理时间, 调到 120s
5. **Step2 Stub6→Stub7 卡数秒**: `_dilate_mask_3d` 旧算法 O(N_valid); 优化版用 cv2.dilate 粗筛 + cKDTree 精检候选点
6. **怀疑某 commit 改了 dino_wrapper API**: 先 grep `det\.` 看实际用了哪些字段, c7c4ff44 只改 scores, masks 处理未动

### 陷阱表 (本次新增)

| 陷阱 | 后果 | 对策 |
| --- | --- | --- |
| realsense-ros 4.58.2 README 与代码不符 (PR #277 后文档过期) | 误以为 `/depth/color/points` 用 aligned depth | 看 `base_realsense_node.cpp:259` 注释, 用 depth_image_proc 替代 |
| depth_image_proc 默认 topic 名 (`depth_registered/image_rect` 等) 与 realsense 不匹配 | 订阅 0 Hz | remap 四个 topic: `depth_registered/image_rect` → `aligned_depth_to_color/image_raw`, `rgb/image_rect_color` → `color/image_raw`, `rgb/camera_info` → `color/camera_info`, `points` → `depth/color/points` |
| depth_image_proc 字段命名 | 下游解析失败? | 不会, depth_image_proc 用 `setPointCloud2FieldsByString(2, "xyz", "rgb")` 与 realsense 一致 |
| depth_image_proc 输出在 color 坐标系 | 与旧 realsense 点云数值差 baseline 视差 | 现有脚本 (`start_open_bottle.py`/`start_insert_straw.py`/`start_grasp_garbage.py`) 本来就假设 `color_optical_frame`, 反而修复了长期 bug |
| TF buffer 默认 cache 10s vs SAM+GraspNet 15s+ | "Lookup would require extrapolation into the past" | `Buffer(cache_time=Duration(seconds=120))` |
| `_dilate_mask_3d` 对所有 valid 点 cKDTree.query | 1280×720 下 2-5s | cv2.dilate 自适应 kernel 粗筛, 仅对候选点 query, 加速 28× |
| 命令笔误: image_topic 和 pointcloud_topic 不同相机 | `pts=0` 但 `dets>0`, 误判为 API bug | 同一相机, 或加 frame_id sanity check |
| colcon `--symlink-install` ament_python 残留 stale build/lib | 改源码不生效 | `rm -rf build/<pkg> install/<pkg>` 后重新 build |

---

## 本次会话修改记录 (相机高延迟修复, 2026-08-26)

### 问题现象

- `start_grasp_graspnet.py` 在 `launch()` 循环里新增机械臂状态检查后, 相机延迟骤增:

```python
robot_mode = self._robot_ctrl.get_robot_mode(self.pick_arm)
if robot_mode == 6:  # Drag
    self._robot_ctrl.stop_drag(self.pick_arm)
if robot_mode == 9:  # Error
    self._robot_ctrl.clear_error_arm(self.pick_arm)
    self._robot_ctrl.enable_arm(self.pick_arm)
```

### `xtrainer_control/robot_control.py` — spin_until_future_complete 双重挂载

- [x] **根因**: 任务节点已由 `main()` 的常驻 `MultiThreadedExecutor` spin (`start_grasp_graspnet.py:1773-1777`), 而 `RobotController._call_service`/`get_robot_mode` 内部调 `rclpy.spin_until_future_complete(self._node, ...)` — 它把同一 node 再挂到 global 单线程 executor 并发 spin。Jazzy 的 `Executor.add_node` 无重复挂载保护 → 相机等高频回调被两个 executor 竞争/双重分发, 时序打乱 → 相机延迟骤增 (与 584 行注释记录的 `spin_once` 坑同源)
- [x] 新增 `_wait_future()`: 若 `node.executor is not None` (已被外部 executor 挂载) 直接轮询 `future.done()` (响应由常驻 executor 完成); 仅独立 CLI 节点 (enable_arms.py 等, 无 executor) 才回退 `spin_until_future_complete`
- [x] 新增 `_get_client()` 按 `(srv_type, service_name)` 缓存 client: 旧代码每次调用 `create_client` 且从不 destroy, `enable_arm` 轮询模式每秒泄漏一个 DDS client
- [x] 兼容性: CLI 入口脚本节点未挂 executor, 行为不变

### 陷阱表 (本次新增)

| 陷阱 | 后果 | 对策 |
| --- | --- | --- |
| 已挂 executor 的节点内调 `rclpy.spin_until_future_complete(node)` / `spin_once(node)` | node 被挂到 global + 常驻两个 executor 并发 spin, 高频回调双重分发 → 相机延迟骤增 | 判断 `node.executor`, 已挂载则只轮询 future; 或服务调用放独立线程+独立回调组 |
| 服务调用每次 create_client 不 destroy | DDS 实体累积, 发现流量渐增 | 按 (类型, 服务名) 缓存 client 复用 |

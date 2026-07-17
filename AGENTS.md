# AGENTS.md — XTrainer 项目记忆

> 最后更新: 2026-07-17

---

## 项目概述

XTrainer 是一个双臂机器人系统，使用 ROS2 进行控制。

### 硬件配置

| 组件 | 数量 | 说明 |
|------|------|------|
| Dobot 机械臂 | 4 | Arm1(左)/Arm2(右) 各 2 个一组，型号 **Nova2** |
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

## MoveIt 控制配置

### 架构概览

```
MoveIt (move_group)
  └─ /Arm1_controller/follow_joint_trajectory (FollowJointTrajectory Action)
  └─ /Arm2_controller/follow_joint_trajectory (FollowJointTrajectory Action)
        ↓
xtrainer_bridge (双臂 FollowJointTrajectory Action Server)
  └─ 逐点调用 ServoJ 服务 (rad→deg 转换, t=相邻dt)
        ↓
dobot_bringup_v4 (cr_robot_ros2_node, 每臂一个, /Arm1 /Arm2 namespace)
  └─ TCP Dashboard (端口 29999) 发送 ServoJ(j1,...,j6,t=...) 命令
  └─ TCP RealTime (端口 30004) 接收关节状态反馈
```

### MoveIt 包 (`moveit_test`)

| 文件 | 说明 |
|------|------|
| `config/moveit_controllers.yaml` | MoveIt controller manager 配置, 指向 `/ArmX_controller/follow_joint_trajectory` |
| `config/joint_limits.yaml` | 关节速度/加速度限制 (控制 TOTG 时间参数化) |
| `config/pilz_cartesian_limits.yaml` | Pilz 规划器笛卡尔限制 |
| `config/x_trainer.urdf.xacro` | URDF 入口 (不含 ros2_control, 避免 action server 冲突) |
| `config/x_trainer.srdf` | 规划组定义: Arm1 (base_link→L1_6), Arm2 (base_link→L2_6) |
| `launch/demo.launch.py` | MoveIt demo 启动 (自定义, 不启动 ros2_control_node) |

### 关键配置值

#### joint_limits.yaml (TOTG 时间参数化)

```yaml
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
# 每个关节 (J1_1~J1_6, J2_1~J2_6):
has_velocity_limits: true
max_velocity: 3.14          # rad/s, ×0.1 scaling = 0.314 rad/s ≈ 18°/s
has_acceleration_limits: true
max_acceleration: 3.14      # rad/s², 限加速度让 TOTG 生成梯形(非bang-bang)
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
|---------|---|---|---|---|---|---|
| 角度范围 (rad) | ±6.28 | ±3.14 | ±2.79 | ±6.28 | ±6.28 | ±6.28 |
| (度) | ±360 | ±180 | ±160 | ±360 | ±360 | ±360 |

URDF effort/velocity 全部设为 `0` (=不限制), 清理 SolidWorks 导出垃圾值。关节级限速由 `joint_limits.yaml` 管理。

### xtrainer_bridge 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_state_publish_rate` | 50.0 | /joint_states 合并发布频率, 与 driver 同频 |
| `servoj_angle_unit` | 'rad' | 'rad'=弧度转度发送, 'deg'=已是度 |
| `servoj_initial_t` | 0.05 | 首 ServoJ 命令的 t 参数 (秒) |
| `servoj_t_min` | 0.004 | t 下限 (文档要求 ≥0.004) |
| `control_frequency` | -1 (已废弃) | 不再驱动插补, 仅向后兼容 |

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
ServoJ(J1,J2,J3,J4,J5,J6,t,lookahead_time,gain)
```

- **J1~J6**: 目标关节角度, 单位 **度** (不是弧度!)
- **t** (可选): 该点位的运行时间, 秒, 范围 [0.02, 3600.0], 默认 0.1
- **lookahead_time** (可选): 前瞻量, 范围 [20.0, 100.0], 默认 50
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

- RealTime TCP (端口 30004) 接收 `q_actual[6]` (度)
- `command.cpp:52`: `current_joint_[i] = deg2Rad(real_time_data_->q_actual[i])` — **转成弧度**发布到 `joint_states_robot`
- 因此 MoveIt 看到的是弧度, 发给 ServoJ 时必须转回度

---

## 官方 SDK 参考

### 路径

```
/opt/Project/XTrainer/SDK/DOBOT_6Axis_ROS2_V4/
```

### 关键文件

| 文件 | 说明 |
|------|------|
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

---

## MoveIt 调优经验教训

### 排错决策树 (运动不正常时)

1. **完全不动**: 检查 action server 是否冲突 (`ros2 action list`), 检查 ServoJ 单位 (rad→deg)
2. **微微动一下**: 99% 是 rad→deg 单位没转 (0.138rad 被当成 0.138°)
3. **能动但卡顿**: 检查轨迹执行策略 (是否本地插补), 检查 t 参数 (是否固定值 vs 相邻dt), 检查 driver TCP 阻塞
4. **连续运动中刹车**: 检查 t 是否远大于发送间隔 (末端减速), 检查 joint_limits (TOTG bang-bang)
5. **MoveIt execute 失败/timeout**: 检查是否有 ros2_control 与 bridge 的 action server 冲突

### 关键陷阱

| 陷阱 | 后果 | 对策 |
|------|------|------|
| ros2_control + bridge 同名 action server | goal/result 路由错乱 | 移除 ros2_control 或用不同 action 名 |
| ServoJ 传弧度不传度 | 几乎不动 | bridge 做 `degrees()` 转换 |
| 本地线性插补 | waypoint 边界速度阶跃 → 卡顿 | 直发 waypoint, 依赖控制器内部插补 |
| t=固定值 ≠ 相邻dt | 末端刹车或跟踪不一致 | t=相邻 time_from_start 差值 |
| URDF velocity/effort 垃圾值 | TOTG 生成激进 bang-bang | URDF 设 0, joint_limits.yaml 管限速 |
| has_acceleration_limits=false | TOTG 无加速度约束 → bang-bang | 设 true + 合理 max_acceleration |
| call_async 无背压 | driver 队列堆积 → 突发下发 → 抖动 | 绝对时刻节拍对齐 (但 fire-and-forget 即可) |
| joint_states 频率 < driver 频率 | /tf 延迟/跳变 | joint_state_publish_rate=50, 与 driver 同频 |
| driver `sleep(0.01)` = `sleep(0)` | CPU 100% 忙轮询 | `sleep_for(1ms)` + `tcpRecv(timeout=100)` |
| `DeclareBooleanLaunchAction` | ImportError (不存在) | 用 `moveit_configs_utils.launch_utils.DeclareBooleanLaunchArg` |

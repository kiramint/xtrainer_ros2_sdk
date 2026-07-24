# moveit_test — XTrainer MoveIt 配置与启动

XTrainer 双臂机器人的 MoveIt2 配置包，包含 URDF 定义、运动学、规划管线、关节限制、控制器管理和整机启动文件。

## 配置

### 文件结构

| 文件 | 说明 |
|------|------|
| `config/x_trainer.urdf.xacro` | URDF 入口（不含 ros2_control，避免 action server 冲突） |
| `config/x_trainer.srdf` | 规划组定义：Arm1 (`base_link`→`L1_6`)、Arm2 (`base_link`→`L2_6`) |
| `config/joint_limits.yaml` | 关节速度/加速度限制（TOTG 时间参数化） |
| `config/kinematics.yaml` | 运动学求解器配置 |
| `config/ompl_planning.yaml` | OMPL 规划管线 + Ruckig 轨迹平滑 |
| `config/pilz_cartesian_limits.yaml` | Pilz 规划器笛卡尔限制 |
| `config/moveit_controllers.yaml` | MoveIt controller manager 配置 |
| `config/sensors_3d.yaml` | 3D 传感器配置（已禁用 octomap） |
| `config/initial_positions.yaml` | 初始关节位置 |
| `config/moveit.rviz` | RViz 配置 |

### 关键配置值

#### 速度缩放

```yaml
default_velocity_scaling_factor: 0.3
default_acceleration_scaling_factor: 0.1
```

#### 关节限制（所有 ARM 关节）

```yaml
has_velocity_limits: true
max_velocity: 3.14            # rad/s
has_acceleration_limits: true
max_acceleration: 3.14        # rad/s²
```

#### 规划管线

```
OMPL → TOTG (时间参数化) → Ruckig (轨迹平滑，bang-bang → jerk 有限 S 曲线) → Validate → Display
```

#### Pilz 笛卡尔限制

```yaml
max_trans_vel: 0.5   # m/s
max_trans_acc: 1.0   # m/s²
max_trans_dec: -2.0  # m/s²
max_rot_vel: 0.785   # rad/s
```

## 启动

### 前置条件

必须先启动 `xtrainer_control start.launch.py`（提供 `/joint_states` + `FollowJointTrajectory` action server）：

```bash
ros2 launch xtrainer_control start.launch.py
```

验证 action server 就绪：

```bash
ros2 action list | grep follow_joint_trajectory
```

### 启动 MoveIt

```bash
# 完整启动（move_group + RViz）
ros2 launch moveit_test demo.launch.py

# 仅启动 move_group（不含 RViz）
ros2 launch moveit_test move_group.launch.py

# 不启动 RViz
ros2 launch moveit_test demo.launch.py use_rviz:=false
```

> [!CAUTION]
> 不要在未启动 `xtrainer_control start.launch.py` 的情况下启动 `demo.launch.py`，否则 move_group 找不到 `follow_joint_trajectory` action server。

> [!NOTE]
> 在 Wayland 下 MoveIt 与 RViz2 可能出现渲染异常，请执行 `export QT_QPA_PLATFORM=xcb` 强制 X11 渲染。

## 架构

```
move_group (moveit_test)
  ├── /Arm1_controller/follow_joint_trajectory (FollowJointTrajectory Action Client)
  └── /Arm2_controller/follow_joint_trajectory (FollowJointTrajectory Action Client)
         ↓
xtrainer_controller (xtrainer_bridge, 独立 node)
  ├── Arm1: call_async ServoJ (degree) → driver → TCP Dashboard
  └── Arm2: call_async ServoJ (degree) → driver → TCP Dashboard
```

> 本包不启动 `ros2_control_node`，避免与 `xtrainer_bridge` 在同名 `/ArmX_controller/follow_joint_trajectory` action server 上冲突。

## 规划组

| 规划组 | 运动链 | 说明 |
|--------|--------|------|
| `Arm1` | `base_link` → `L1_6` | 左臂 (6 自由度) |
| `Arm2` | `base_link` → `L2_6` | 右臂 (6 自由度) |

## 编译

```bash
colcon build --packages-select moveit_test
source install/setup.bash
```

## 调优经验

| 陷阱 | 后果 | 对策 |
|------|------|------|
| ros2_control 与 bridge 同名 action server | goal/result 路由错乱 | 移除 ros2_control |
| URDF velocity/effort 垃圾值 | TOTG 生成激进 bang-bang | URDF 设 0，`joint_limits.yaml` 管理 |
| `has_acceleration_limits=false` | Jazzy TOTG 直接拒绝运行 | 必须 true + 合理 `max_acceleration` |
| TOTG 无 Ruckig 平滑 | bang-bang → jerk 无穷大 → 卡顿 | 加 `AddRuckigTrajectorySmoothing` |
| 传感器配置引用不存在 topic | 持续报错 | 禁用 octomap（`sensors: []`） |

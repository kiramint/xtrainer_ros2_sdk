# xtrainer_control — XTrainer 控制程序

XTrainer 双臂机器人系统的顶层控制包，负责整机启动、机械臂状态控制、手眼标定。

## Launch 文件

| 文件 | 说明 |
|------|------|
| `start.launch.py` | **主启动**：双臂驱动 + 夹爪 + 三台相机 + 标定 TF 发布 + 自动使能 |
| `enable.launch.py` | 一键使能双臂 |
| `disable.launch.py` | 一键去使能双臂 |
| `enable_and_drag.launch.py` | 一键使能 + 拖拽示教双臂 |
| `clear_error.launch.py` | 清除双臂错误状态 |
| `calibrate_top.launch.py` | 头顶相机手眼标定（eye_on_base） |
| `calibrate_left.launch.py` | 左手相机手眼标定（eye_in_hand） |
| `calibrate_right.launch.py` | 右手相机手眼标定（eye_in_hand） |

## Start Launch 详细流程

`start.launch.py` 按顺序启动：

1. **`xtrainer_driver`** — `cr_robot_ros2/xtrainer.launch.py`（双臂驱动 + robot_state_publisher + bridge）
2. **`gripper_node`** — 夹爪节点（单节点管理双臂，话题：`/gripper/left/*`、`/gripper/right/*`）
3. **RealSense 相机** — `camera_top`、`camera_left`、`camera_right`
4. **标定 TF 发布** — 自动检测 `~/.ros2/easy_handeye2/calibrations/` 下的标定结果并加载
5. **延迟 3s 自动使能** — 确保驱动节点 TCP 连接建立后执行 `enable_arms`

## Entry Points

| 命令 | 功能 | 对应 launch |
|------|------|-------------|
| `enable_arms` | 双臂使能（PowerOn→轮询 RobotMode→EnableRobot，幂等） | `enable.launch.py` |
| `disable_arms` | 双臂去使能 | `disable.launch.py` |
| `enable_and_drag` | 双臂使能 + 拖拽示教 | `enable_and_drag.launch.py` |
| `clear_error` | 清除双臂错误 | `clear_error.launch.py` |

## robot_control.py API

`xtrainer_control.robot_control` 模块提供机械臂状态管理函数：

| 函数 | 说明 | 参数 |
|------|------|------|
| `enable_arm(node, ns, timeout=30)` | 单臂使能（PowerOn→轮询 RobotMode→EnableRobot），已使能则跳过 | node, arm_namespace, timeout |
| `disable_arm(node, ns, timeout=10)` | 单臂去使能 | node, arm_namespace, timeout |
| `start_drag(node, ns, timeout=5)` | 单臂拖拽示教开 | node, arm_namespace, timeout |
| `stop_drag(node, ns, timeout=5)` | 单臂拖拽示教关 | node, arm_namespace, timeout |
| `enable_all(node, nss, timeout=10)` | 多臂使能，默认 `["Arm1","Arm2"]` | node, namespaces, timeout |
| `disable_all(node, nss, timeout=10)` | 多臂去使能 | node, namespaces, timeout |

### 使能流程

```
PowerOn → 等待 RobotMode ≥ 4 (初始化为 1) → EnableRobot
```

- RobotMode 返回值：1=初始化, 2=抱闸松开, 3=未上电, 4=未使能, 5=使能空闲, 6=拖拽模式, 7=运行中, 9=报错
- 幂等：若 `RobotMode ≥ 5` 则跳过使能

## 标定

### 标定配置

| 相机 | Launch | 标定类型 | 标定名 | effector | tracking_base_frame |
|------|--------|----------|--------|----------|---------------------|
| camera_top | `calibrate_top.launch.py` | `eye_on_base` | `top_cam_cal` | L1_6 | camera_top_link |
| camera_left | `calibrate_left.launch.py` | `eye_in_hand` | `left_cam_cal` | L1_6 | camera_left_link |
| camera_right | `calibrate_right.launch.py` | `eye_in_hand` | `right_cam_cal` | L2_6 | camera_right_link |

标定产物路径：`~/.ros2/easy_handeye2/calibrations/<name>.calib`

### ArUco 标定板参数

- marker_id: 99
- marker_size: 0.078 (7.8cm)
- corner_refinement: LINES

### 标定流程

```bash
# 1. 启动机器人驱动
ros2 launch cr_robot_ros2 xtrainer.launch.py

# 2. 进入机器人示教模式
ros2 launch xtrainer_control enable_and_drag.launch.py

# 3. 启动标定程序
ros2 launch xtrainer_control calibrate_top.launch.py
# 或
ros2 launch xtrainer_control calibrate_left.launch.py
# 或
ros2 launch xtrainer_control calibrate_right.launch.py
```

> [!NOTE]
> 标定程序只会启动单个需要标定的相机。正式运行时启动三个相机，USB 带宽将达到 15Gbps，建议将相机分别插在不同的 USB 根集线器上。

## 编译

```bash
colcon build --packages-select xtrainer_control
source install/setup.bash

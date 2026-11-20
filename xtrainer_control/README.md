# xtrainer_control — XTrainer 控制程序

XTrainer 双臂机器人系统的顶层控制包，负责整机启动、机械臂状态控制、手眼标定。

## Launch 文件

| 文件 | 说明 |
|------|------|
| `start.launch.py` | **主启动**：双臂驱动 + 夹爪 + 三台相机 + 标定 TF 发布。抱闸由驱动进程负责 |
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

机械臂抱闸不在这里做。`cr_robot_ros2_node` 连上控制器后解锁，进程退出时锁定。下面的 `enable_arms` 用于中途报警后再使能，或驱动参数 `auto_enable:=false` 时手动解锁。

## Entry Points

| 命令 | 功能 | 对应 launch |
|------|------|-------------|
| `enable_arms` | 按 RobotMode 重试上电/使能，直到抱闸松开（默认每臂 90s） | `enable.launch.py` |
| `disable_arms` | 双臂去使能 | `disable.launch.py` |
| `enable_and_drag` | 双臂使能 + 拖拽示教 | `enable_and_drag.launch.py` |
| `clear_error` | 清除双臂错误 | `clear_error.launch.py` |

## robot_control.py API

这些是 `RobotController` 的方法，不是模块级函数。

| 方法 | 说明 | 默认超时 |
|------|------|----------|
| `enable_arm(ns)` | 按模式重试：报警先清错，未上电则 PowerOn，模式 2 先逐轴锁抱闸，模式 4 再 EnableRobot。模式进入 5/6/7/8/10 才算成功 | 90s |
| `disable_arm(ns)` | 拖拽模式先 StopDrag，再 DisableRobot。模式 3/4 视为已经抱闸 | 10s |
| `start_drag(ns)` | 打开拖拽示教 | 5s |
| `stop_drag(ns)` | 关闭拖拽示教 | 5s |
| `enable_all(nss)` | 默认 `["Arm1","Arm2"]`。超时是每条臂各自的上限 | 90s |
| `disable_all(nss)` | 多臂去使能 | 10s |

### 使能流程

```
读 RobotMode
  5/6/7/8/10 → 已经解锁
  9/11       → ClearError，再看
  0/1        → 控制柜还在起来，继续等
  3          → PowerOn，等约 10 秒，期间不重复下发
  2          → BrakeControl(1..6, 0)，再看
  4          → EnableRobot，确认模式真的变了
```

- RobotMode：1=初始化, 2=有抱闸松开, 3=未上电, 4=未使能, 5=使能空闲, 6=拖拽, 7=运行, 8=点动, 9=报警, 10=暂停, 11=碰撞
- 控制器拒绝某一步，或 Dashboard 暂时连不上，会在超时内重试。一次失败不会让整次使能直接返回

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

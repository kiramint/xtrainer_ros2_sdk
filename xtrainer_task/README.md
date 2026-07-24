# xtrainer_task — XTrainer Demo 任务

基于 MoveItPy 的 XTrainer 双臂机器人任务层控制包，提供规划、执行、视觉检测与 RViz 可视化一体化 Demo。

## 架构

```
xtrainer_task (MoveItPy 进程内)
  ├── PlanningComponent (规划)
  ├── TrajectoryExecutionManager (执行)
  ├── PlanningSceneMonitor (场景监控)
  └── 自定义:
      ├── GroundingDINO 目标检测 (dino_wrapper.py)
      ├── 深度图→3D 坐标转换 (start.py)
      ├── 轨迹可视化 (robot_move.py, /display_planned_path)
      └── 检测点 Marker (robot_move.py, /detection_marker)
```

> MoveItPy 是 move_group 的**平级替代**（非 client），运行在独立进程中，不依赖 move_group。

## Entry Points

| 命令 | 说明 | 对应 launch |
|------|------|-------------|
| `start` | **开瓶盖 Demo 主程序**：检测→规划→夹取→拧开 | `start.launch.py` |
| `dino_test` | GroundingDINO 目标检测测试 | 直接 `ros2 run` |
| `read_pose` | 读取当前关节姿态并打印 | `read_pose.launch.py` |
| `goto_pose` | 移动机器人到零位姿态 | `goto_pose.launch.py` |

## Launch 文件

| 文件 | 说明 |
|------|------|
| `start.launch.py` | 开瓶盖 Demo 启动（含 MoveItPy + RViz，可禁用 RViz） |
| `read_pose.launch.py` | 读取姿态启动（含 MoveItPy） |
| `goto_pose.launch.py` | 归零位启动（含 MoveItPy） |

### start.launch.py 参数

```bash
# 禁用 RViz（仅运行任务逻辑）
ros2 launch xtrainer_task start.launch.py use_rviz:=false
```

## 模块

### robot_move.py — 运动控制

基于 MoveItPy `PlanningComponent` 的运动控制封装：

| 方法 | 说明 |
|------|------|
| `plan_joints(target)` | 关节空间规划 |
| `plan_pose(target, frame)` | 笛卡尔空间规划 |
| `execute()` | 执行已规划轨迹 |
| `plan_and_execute_joints(target)` | 规划并执行（关节空间） |
| `plan_and_execute_pose(target, frame)` | 规划并执行（笛卡尔空间） |

规划成功后自动推送轨迹到 RViz（`/display_planned_path`）。

### dino_wrapper.py — 目标检测

封装 GroundingDINO，订阅相机话题进行目标检测：

```bash
ros2 run xtrainer_task dino_test --ros-args \
  -p prompt:="bottle" \
  -p image_topic:="/camera/camera_top/color/image_raw"
```

### start.py — 开瓶盖 Demo

完整任务流程：目标检测 → 3D 定位 → 运动规划 → 夹取 → 拧开瓶盖。

依赖 GroundingDINO + SAM2 进行视觉检测。

## Topics

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | 发布 | 规划轨迹（RViz 显示） |
| `/detection_marker` | `visualization_msgs/Marker` | 发布 | 检测点红色小球 Marker |

## RViz 配置

`config/xtrainer.rviz` 预配置：

- `PlanningScene` — 订阅 `/moveit_cpp/monitored_planning_scene` 显示机器人模型
- `Trajectory` — 订阅 `/display_planned_path` 显示规划轨迹
- `Marker` — 订阅 `/detection_marker` 显示检测点

> 这是 `PlanningSceneDisplay`，不是 `MotionPlanningDisplay`（后者需要 move_group）。

## 配置

`config/moveit_cpp.yaml` — MoveItCpp 参数：

- `planning_scene_monitor_options` — 场景监控
- `plan_request_params` — 规划请求（planner_id, planning_pipeline 等）
- `ompl_rrtc` — RRTConnect 参数
- `pilz_ptp` — Pilz PTP 参数

## 编译

```bash
colcon build --packages-select xtrainer_task
source install/setup.bash
```

## 前置条件

1. 启动 `xtrainer_control start.launch.py`（提供 `/joint_states` + FollowJointTrajectory action server）
2. 如需目标检测功能，安装 GroundingDINO 与 SAM2（参考根目录 README.md）

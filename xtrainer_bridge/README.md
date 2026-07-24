# xtrainer_bridge — XTrainer ROS2 兼容层

双臂桥接包，为 MoveIt 提供统一的 `FollowJointTrajectory` action server 和合并后的 `/joint_states`。拆分为两个独立 node 以保证联合状态发布的稳定性。

## 架构

```
Arm1 driver (/Arm1/joint_states_robot)
Arm2 driver (/Arm2/joint_states_robot)
         ↓                        ↓
xtrainer_joint_states (合并, 事件驱动, 无 timer)
         ↓
/joint_states (frame_id: base_link, 12 关节 + 哑关节)
         ↓
move_group (MoveIt)
  ├── /Arm1_controller/follow_joint_trajectory → xtrainer_controller
  └── /Arm2_controller/follow_joint_trajectory → xtrainer_controller
         ↓
xtrainer_controller (逐点 ServoJ, rad→deg 转换, fire-and-forget)
  ├── Arm1: call_async /Arm1/dobot_bringup_ros2/srv/ServoJ (度)
  └── Arm2: call_async /Arm2/dobot_bringup_ros2/srv/ServoJ (度)
```

## 节点

### `xtrainer_joint_states` — 关节状态合并

事件驱动合并双臂 joint_states，无 timer，`rclpy.spin` 单线程。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm1_joint_names` | `['J1_1'..'J1_6']` | 左臂关节名 |
| `arm2_joint_names` | `['J2_1'..'J2_6']` | 右臂关节名 |
| `arm1_joint_state_topic` | `/Arm1/joint_states_robot` | 左臂状态话题 |
| `arm2_joint_state_topic` | `/Arm2/joint_states_robot` | 右臂状态话题 |
| `dummy_joint_names` | `['J1_7','J1_8','J2_7','J2_8','J3_1'~'J3_6','J4_1'~'J4_6']` | 哑关节（无驱动，发布 0 值消除 MoveIt warning） |

### `xtrainer_controller` — FollowJointTrajectory→ServoJ 桥接

双臂 FollowJointTrajectory Action Server，MultiThreadedExecutor + ReentrantCallbackGroup。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm1_servoj_service` | `/Arm1/dobot_bringup_ros2/srv/ServoJ` | 左臂 ServoJ 服务 |
| `arm2_servoj_service` | `/Arm2/dobot_bringup_ros2/srv/ServoJ` | 右臂 ServoJ 服务 |

#### 轨迹执行逻辑

- **直发 waypoint**，不本地插补（依赖控制器内部插补）
- **rad→deg 转换**：MoveIt 规划为弧度，Dobot ServoJ 协议要求度
- **t = 相邻 time_from_start 差值**（首点 t=0.05）
- **绝对时刻节拍对齐**（`time.sleep(cur_tfs - elapsed)`）
- **fire-and-forget**（`call_async`，不等 driver TCP echo）

## Topics & Actions

### `xtrainer_joint_states`

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/Arm1/joint_states_robot` | `sensor_msgs/JointState` | 订阅 | 左臂关节状态 |
| `/Arm2/joint_states_robot` | `sensor_msgs/JointState` | 订阅 | 右臂关节状态 |
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 合并后的关节状态（含哑关节） |

### `xtrainer_controller`

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/Arm1_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Action Server | 左臂轨迹执行 |
| `/Arm2_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Action Server | 右臂轨迹执行 |

## Entry Points

| 命令 | 说明 |
|------|------|
| `xtrainer_joint_states` | 启动 joint_states 合并节点 |
| `xtrainer_controller` | 启动 FollowJointTrajectory action server |
| `xtrainer_bridge_node` | 旧版单节点（已弃用，保留兼容） |

## 编译

```bash
colcon build --packages-select xtrainer_bridge
source install/setup.bash
```

## 关键注意

- **ServoJ 单位**：controller 内部做 `rad→deg` 转换，确保调用方（MoveIt）使用弧度
- **action server 冲突**：不要同时启动 `ros2_control_node`，否则同名 action server 导致路由错乱
- **多线程隔离**：两个 node 为独立进程，joint_states 发布不受 ServoJ 调用影响

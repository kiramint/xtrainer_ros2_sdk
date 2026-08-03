# dobot_bringup_v4 — Dobot Nova2 机械臂 ROS2 驱动

基于 Dobot 官方 Nova2 ROS2 SDK（`cr_robot_ros2`）修改的双臂驱动包，支持 ROS2 Jazzy。

## 修改内容

| 修改项 | 说明 |
|--------|------|
| 删除 `robot_number` | 原 `kRobotName` 逻辑导致 namespace 失效，删掉后所有 topic/service 改为相对路径 |
| MultiThreadedExecutor | 替代 `while`+`spin_some`，避免 ServoJ 回调阻塞 joint_states 发布 |
| 三组 callback group 隔离 | `timer_cb_group` (joint_states 发布)、`servo_cb_group_` (ServoJ/ServoP)、默认 group (~70 个其他服务)，三者互不阻塞 |
| Dashboard TCP 互斥锁 | `tcp_mutex_` 保护 `dash_board_tcp_` 的 send+recv，防止不同 callback group 的服务并发写同一 socket 导致命令交错损坏 |
| `doTcpCmd` 日志可控 | `#define DEBUG 0` 硬编码关闭每次 TCP 命令的 stdout 输出 (原 3 条/call × 33Hz × 双臂 = 198 条/s)，开发时改 `1` 即可恢复 |
| RealTime 数据竞态修复 | `recvTask` 改为先写入栈上 local buffer，再在 `mutex_` 下拷贝到 `real_time_data_`；`getRealData()` 返回值拷贝 (不再返回 shared_ptr) |
| TCP 忙循环修复 | `tcpRecv(timeout=0)` 改为 `timeout=100ms`（select 阻塞），`sleep(0.01)` 改为 `sleep_for(1ms)` |
| `joint_states_robot` 发布 | 50Hz wall_timer (独立 callback group)，角度为弧度（`command.cpp` 做 deg→rad 转换） |

## Launch 文件

| 文件 | 说明 |
|------|------|
| `xtrainer.launch.py` | **双臂启动**：Arm1 + Arm2 驱动节点、`robot_state_publisher`、`xtrainer_joint_states`、`xtrainer_controller` |
| `dobot_bringup_ros2.launch.py` | **单臂启动**：仅启动一个驱动节点（通过环境变量 `IP_address`、`DOBOT_TYPE` 配置） |

## 配置

`config/param.json` 中定义机械臂连接参数：

```json
{
  "node_info": [
    {
      "ip_address": "192.168.1.6",
      "robot_type": "nova2",
      "robot_node_name": "dobot_bringup_ros2",
      "trajectory_duration": 500,
      "joint_names": ["J1_1", "J1_2", "J1_3", "J1_4", "J1_5", "J1_6"]
    },
    {
      "ip_address": "192.168.1.8",
      "robot_type": "nova2",
      "robot_node_name": "dobot_bringup_ros2",
      "trajectory_duration": 500,
      "joint_names": ["J2_1", "J2_2", "J2_3", "J2_4", "J2_5", "J2_6"]
    }
  ]
}
```

IP 地址可通过环境变量覆写：

```bash
ARM1_IP=192.168.1.10 ARM2_IP=192.168.1.11 ros2 launch cr_robot_ros2 xtrainer.launch.py
```

## Topics & Services

### Published Topics

| Topic | Type | 说明 |
|-------|------|------|
| `joint_states_robot` | `sensor_msgs/JointState` | 关节状态（弧度），50Hz |
| `robot_status` | `dobot_msgs_v4/RobotStatus` | 机械臂状态 |

### Services（全部在 `dobot_bringup_ros2/srv/` 下）

常用服务：

| Service | 说明 |
|---------|------|
| `PowerOn` | 上电 |
| `EnableRobot` | 使能 |
| `DisableRobot` | 去使能 |
| `ClearError` | 清除错误 |
| `RobotMode` | 查询模式 |
| `StartDrag` | 进入拖拽示教 |
| `StopDrag` | 退出拖拽示教 |
| `ServoJ` | 关节空间伺服（角度，单位**度**） |
| `ServoP` | 笛卡尔空间伺服 |
| `MovJ` / `MovL` | 关节/直线运动 |

> [!NOTE]
> `ServoJ` 的 J1~J6 参数单位为**度**（不是弧度）。驱动发布 `joint_states_robot` 时做 deg→rad 转换，但 `ServoJ` 调用方必须传度。`xtrainer_bridge` 已处理此转换。

## 编译

```bash
colcon build --packages-select cr_robot_ros2 dobot_msgs_v4
source install/setup.bash
```

> [!NOTE]
> `data_list.h` 随 SDK 版本更新，请确保与 DobotStudio Pro 版本一致。

## 架构

```
cr_robot_ros2_node (namespace /ArmX)
│
├── recvTask 线程 (独立后台线程)
│   ├── TCP RealTime (端口 30004) ← 控制器每 8ms (125Hz) 推送 1440 字节二进制
│   │   └── q_actual[6] (度) → 栈上 local buffer → mutex_ → current_joint_[] (弧度) + real_time_data_
│   └── 自动重连 Dashboard + RealTime
│
├── MultiThreadedExecutor
│   ├── timer_cb_group (MutuallyExclusive):
│   │   └── wall_timer 50Hz → getCurrentJointStatus(mutex_ memcpy) → publish joint_states_robot
│   │                                      → getToolVectorActual → publish ToolVectorActual
│   │                                      → publish RobotStatus
│   │
│   ├── servo_cb_group_ (MutuallyExclusive):
│   │   └── ServoJ/ServoP → doTcpCmd(tcp_mutex_ lock) → TCP Dashboard send+echo (~10-15ms)
│   │
│   └── 默认 group (MutuallyExclusive):
│       └── ~70 个其他服务 → doTcpCmd(tcp_mutex_ lock) → TCP Dashboard send+echo
│
└── pubFeedBackInfo 线程 (独立后台线程, 100Hz)
    └── getRealData() → mutex_ 下值拷贝 RealTimeData → JSON → publish FeedInfo
```

## 参考

- 官方 SDK: `/opt/Project/XTrainer/SDK/DOBOT_6Axis_ROS2_V4/`
- TCP 协议文档: `/opt/Project/TCP-IP-Protocol-6AXis-V4/Dobot TCP_IP二次开发接口文档V4.5.1_20240815_cn.md`

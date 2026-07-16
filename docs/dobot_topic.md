# Dobot ROS2 SDK — 话题与服务文档

> 项目根目录: `/opt/Project/XTrainer/SDK/dobot_ros2_sdk`
> 驱动节点: `dobot_bringup_ros2` (CRRobotRos2, 对应 Dobot CR 系列机械臂)

---

## 一、话题 (Topics)

### 1.1 `joint_states_robot`

| 属性 | 值 |
|------|-----|
| 消息类型 | `sensor_msgs/msg/JointState` |
| 发布频率 | 由 `JointStatePublishRate` 参数控制 (默认 10 Hz) |
| frame_id | `dummy_link` |
| 定义位置 | `main.cpp` main loop |

**功能**: 发布机械臂 6 个关节的实时位置（弧度）。关节名称可通过 `param.json` 的 `joint_names` 字段配置，与 URDF 保持一致。

**消息字段**:

- `name[]` — 关节名称列表 (如 `["J1_1","J1_2","J1_3","J1_4","J1_5","J1_6"]`)
- `position[]` — 对应关节角度 (rad)
- `velocity[]` — 未填充
- `effort[]` — 未填充

---

### 1.2 `dobot_msgs_v4/msg/RobotStatus`

| 属性 | 值 |
|------|-----|
| 消息类型 | `dobot_msgs_v4/msg/RobotStatus` |
| 发布频率 | 由 `JointStatePublishRate` 控制 (默认 10 Hz) |
| 定义位置 | `main.cpp` main loop |

**功能**: 发布机械臂的使能与连接状态。

**消息字段**:

- `is_enable` (bool) — 机械臂是否已使能
- `is_connected` (bool) — 与控制器 TCP 连接是否正常

---

### 1.3 `dobot_msgs_v4/msg/ToolVectorActual`

| 属性 | 值 |
|------|-----|
| 消息类型 | `dobot_msgs_v4/msg/ToolVectorActual` |
| 发布频率 | 由 `JointStatePublishRate` 控制 (默认 10 Hz) |
| 定义位置 | `main.cpp` main loop |

**功能**: 发布工具端实际位姿（TCP 笛卡尔坐标）。

**消息字段**:

- `x`, `y`, `z` (float64) — TCP 位置 (mm)
- `rx`, `ry`, `rz` (float64) — TCP 姿态 (Euler 角, rad)

---

### 1.4 `{robot_node_name}/dobot_bringup_ros2/msg/FeedInfo`

| 属性 | 值 |
|------|-----|
| 消息类型 | `std_msgs/msg/String` (实际为 JSON 字符串) |
| 发布频率 | 100 Hz |
| 定义位置 | `cr_robot_ros2.cpp` `pubFeedBackInfo()` |

**功能**: 高频发布完整的实时反馈数据，包含约 60 个字段，是机械臂状态获取的核心数据源。

**JSON 字段**:

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `len` | int | 数据包长度 |
| `digital_input_bits` | int | 数字输入位状态 |
| `digital_outputs` | int | 数字输出状态 |
| `robot_mode` | int | 机器人模式 (9=错误状态) |
| `controller_timer` | int | 控制器时钟 |
| `run_time` | int | 运行时间 |
| `test_value` | int | 测试值 |
| `safety_mode` | int | 安全模式 |
| `speed_scaling` | float | 速度缩放比例 (0~1) |
| `linear_momentum_norm` | float | 线动量范数 |
| `v_main` | float | 主电压 (V) |
| `v_robot` | float | 机器人电压 (V) |
| `i_robot` | float | 机器人电流 (A) |
| `program_state` | int | 程序运行状态 |
| `safety_status` | int | 安全状态 |
| `tool_accelerometer_values` | float[3] | 工具加速度计值 |
| `elbow_position` | float[3] | 肘部位置 |
| `elbow_velocity` | float[3] | 肘部速度 |
| `q_target` | float[6] | 关节目标角度 (rad) |
| `qd_target` | float[6] | 关节目标速度 (rad/s) |
| `qdd_target` | float[6] | 关节目标加速度 (rad/s²) |
| `i_target` | float[6] | 关节目标电流 (A) |
| `m_target` | float[6] | 关节目标力矩 (N·m) |
| `q_actual` | float[6] | 关节实际角度 (rad) |
| `qd_actual` | float[6] | 关节实际速度 (rad/s) |
| `i_actual` | float[6] | 关节实际电流 (A) |
| `i_control` | float[6] | 关节控制电流 (A) |
| `m_actual` | float[6] | 关节实际力矩 (N·m) |
| `tool_vector_actual` | float[6] | 工具端实际位姿 [x,y,z,rx,ry,rz] |
| `tool_vector_target` | float[6] | 工具端目标位姿 |
| `TCP_speed_actual` | float[6] | TCP 实际速度 |
| `TCP_speed_target` | float[6] | TCP 目标速度 |
| `TCP_force` | float[6] | TCP 力/力矩 |
| `motor_temperatures` | float[6] | 电机温度 (°C) |
| `joint_modes` | float[6] | 关节模式 |
| `v_actual` | float[6] | 实际电压 |
| `handtype` | float[4] | 手爪类型信息 |
| `userCoordinate` | int | 用户坐标系编号 |
| `toolCoordinate` | int | 工具坐标系编号 |
| `isRunQueuedCmd` | bool | 是否运行队列命令 |
| `isPauseCmdFlag` | bool | 是否暂停标志 |
| `velocityRatio` | float | 速度比例 |
| `accelerationRatio` | float | 加速度比例 |
| `jerkRatio` | float | 加加速度比例 |
| `xyzVelocityRatio` | float | XYZ 线速度比例 |
| `rVelocityRatio` | float | R 旋转速度比例 |
| `xyzAccelerationRatio` | float | XYZ 线加速度比例 |
| `rAccelerationRatio` | float | R 旋转加速度比例 |
| `xyzJerkRatio` | float | XYZ 加加速度比例 |
| `rJerkRatio` | float | R 加加速度比例 |
| `BrakeStatus` | int | 抱闸状态 |
| `EnableStatus` | int | 使能状态 |
| `DragStatus` | int | 拖拽状态 |
| `RunningStatus` | int | 运行状态 |
| `ErrorStatus` | int | 错误状态 |
| `JogStatus` | int | Jog 状态 |
| `RobotType` | int | 机器人类型 |
| `DragButtonSignal` | int | 拖拽按钮信号 |
| `EnableButtonSignal` | int | 使能按钮信号 |
| `RecordButtonSignal` | int | 录制按钮信号 |
| `ReappearButtonSignal` | int | 复现按钮信号 |
| `JawButtonSignal` | int | 手爪按钮信号 |
| `SixForceOnline` | int | 六维力传感器在线状态 |
| `CollisionStates` | int | 碰撞状态 |
| `ArmApproachState` | int | 臂接近状态 |
| `J4ApproachState` | int | J4 接近状态 |
| `J5ApproachState` | int | J5 接近状态 |
| `J6ApproachState` | int | J6 接近状态 |
| `vibrationDisZ` | float | Z 向振动位移 |
| `currentCommandId` | int | 当前命令 ID |
| `load` | float | 负载质量 (kg) |
| `centerX` | float | 负载质心 X (mm) |
| `centerY` | float | 负载质心 Y (mm) |
| `centerZ` | float | 负载质心 Z (mm) |
| `user` | float[6] | 用户坐标系参数 |
| `tool` | float[6] | 工具坐标系参数 |
| `TraceIndex` | int | 轨迹复现索引 |
| `SixForceValue` | float[6] | 六维力传感器值 |
| `TargetQuaternion` | float[4] | 目标四元数 |
| `ActualQuaternion` | float[4] | 实际四元数 |
| `AutoManualMode` | int | 手自动模式 (0:未开启, 1:manual, 2:auto) |

---

## 二、服务 (Services)

所有服务均位于 `{robot_node_name}/dobot_bringup_ros2/srv/` 命名空间下，共约 100+ 个服务，按功能分类如下。

### 2.1 使能与控制类

| 服务名 | 功能 |
|--------|------|
| `EnableRobot` | 使能机械臂 |
| `DisableRobot` | 下使能机械臂 |
| `ClearError` | 清除错误 |
| `PowerOn` | 上电 |
| `ResetRobot` | 复位机械臂 |
| `EmergencyStop` | 紧急停止 |
| `Stop` | 停止运动 |
| `Pause` | 暂停运动 |
| `Continue` | 继续运动 |
| `RequestControl` | 请求控制权 |
| `RunScript` | 运行脚本 |
| `RobotMode` | 获取/设置机器人模式 |

### 2.2 运动控制类

| 服务名 | 功能 |
|--------|------|
| `MovJ` | 关节空间点到点运动 |
| `MovL` | 笛卡尔空间直线运动 |
| `MovJIO` | 带 IO 触发的关节运动 |
| `MovLIO` | 带 IO 触发的直线运动 |
| `Arc` | 圆弧运动 |
| `Circle` | 整圆运动 |
| `MoveJog` | Jog 点动 |
| `StopMoveJog` | 停止 Jog |
| `RelJointMovJ` | 相对关节运动 |
| `RelMovJTool` | 工具坐标系下相对关节运动 |
| `RelMovLTool` | 工具坐标系下相对直线运动 |
| `RelMovJUser` | 用户坐标系下相对关节运动 |
| `RelMovLUser` | 用户坐标系下相对直线运动 |
| `ServoJ` | 关节空间伺服模式 |
| `ServoP` | 笛卡尔空间伺服模式 |
| `RunTo` | 运行到指定位置 |
| `StartPath` | 开始轨迹路径 |
| `GetStartPose` | 获取起始位姿 |
| `GetCurrentCommandId` | 获取当前命令 ID |
| `CheckOddMovL` | 检查直线运动奇点 |
| `CheckOddMovJ` | 检查关节运动奇点 |
| `CheckOddMovC` | 检查圆弧运动奇点 |

### 2.3 运动学类

| 服务名 | 功能 |
|--------|------|
| `PositiveKin` | 正运动学 (关节角 → 笛卡尔位姿) |
| `InverseKin` | 逆运动学 (笛卡尔位姿 → 关节角) |
| `GetPose` | 获取当前笛卡尔位姿 |
| `GetAngle` | 获取当前关节角 |

### 2.4 运动参数配置类

| 服务名 | 功能 |
|--------|------|
| `SpeedFactor` | 设置全局速度比例 (0~100) |
| `AccJ` | 设置关节加速度 |
| `AccL` | 设置直线加速度 |
| `VelJ` | 设置关节速度 |
| `VelL` | 设置直线速度 |
| `CP` | 设置平滑过渡比例 |
| `SetPayload` | 设置负载参数 |
| `SetBackDistance` | 设置回退距离 |
| `SetPostCollisionMode` | 设置碰撞后模式 |

### 2.5 数字 IO 类

| 服务名 | 功能 |
|--------|------|
| `DO` | 设置本体数字输出（队列执行） |
| `DOInstant` | 设置本体数字输出（立即执行） |
| `DI` | 读取本体数字输入 |
| `GetDO` | 获取本体数字输出状态 |
| `DOGroup` | 设置本体数字输出组 |
| `DIGroup` | 读取本体数字输入组 |
| `DOGroupDEC` | 设置本体数字输出组（十进制） |
| `GetDOGroupDEC` | 获取本体数字输出组（十进制） |
| `DIGroupDEC` | 读取本体数字输入组（十进制） |

### 2.6 工具 IO 类

| 服务名 | 功能 |
|--------|------|
| `ToolDO` | 设置工具数字输出（队列执行） |
| `ToolDOInstant` | 设置工具数字输出（立即执行） |
| `ToolDI` | 读取工具数字输入 |
| `GetToolDO` | 获取工具数字输出状态 |

### 2.7 模拟 IO 类

| 服务名 | 功能 |
|--------|------|
| `AO` | 设置模拟输出（队列执行） |
| `AOInstant` | 设置模拟输出（立即执行） |
| `AI` | 读取模拟输入 |
| `ToolAI` | 读取工具模拟输入 |
| `GetAO` | 获取模拟输出状态 |

### 2.8 扩展 IO 类

| 服务名 | 功能 |
|--------|------|
| `GetInputBool` | 读取扩展输入 (bool) |
| `GetInputInt` | 读取扩展输入 (int) |
| `GetInputFloat` | 读取扩展输入 (float) |
| `GetOutputBool` | 读取扩展输出 (bool) |
| `GetOutputInt` | 读取扩展输出 (int) |
| `GetOutputFloat` | 读取扩展输出 (float) |
| `SetOutputBool` | 设置扩展输出 (bool) |
| `SetOutputInt` | 设置扩展输出 (int) |
| `SetOutputFloat` | 设置扩展输出 (float) |

### 2.9 坐标系与工具类

| 服务名 | 功能 |
|--------|------|
| `User` | 切换用户坐标系 |
| `Tool` | 切换工具坐标系 |
| `SetUser` | 设置用户坐标系参数 |
| `SetTool` | 设置工具坐标系参数 |
| `CalcUser` | 计算用户坐标系 |
| `CalcTool` | 计算工具坐标系 |
| `SetTool485` | 设置工具 485 参数 |
| `SetToolPower` | 设置工具电源 |
| `SetToolMode` | 设置工具模式 |

### 2.10 碰撞安全类

| 服务名 | 功能 |
|--------|------|
| `SetCollisionLevel` | 设置碰撞等级 |
| `EnableSafeSkin` | 启用安全皮肤 |
| `SetSafeSkin` | 设置安全皮肤参数 |
| `SetSafeWallEnable` | 设置安全墙使能 |
| `SetWorkZoneEnable` | 设置工作区域使能 |

### 2.11 拖拽示教类

| 服务名 | 功能 |
|--------|------|
| `StartDrag` | 开始拖拽示教 |
| `StopDrag` | 停止拖拽示教 |
| `DragSensivity` | 设置拖拽灵敏度 |

### 2.12 力控类 (Force Control)

| 服务名 | 功能 |
|--------|------|
| `EnableFTSensor` | 启用力传感器 |
| `SixForceHome` | 六维力传感器归零 |
| `GetForce` | 获取力传感器数据 |
| `ForceDriveMode` | 设置力驱动模式 |
| `ForceDriveSpeed` | 设置力驱动速度 |
| `FCForceMode` | 力控力模式 |
| `FCSetForce` | 设置力控目标力 |
| `FCSetDeviation` | 设置力控偏差 |
| `FCSetForceLimit` | 设置力控力限制 |
| `FCSetMass` | 设置力控质量 |
| `FCSetStiffness` | 设置力控刚度 |
| `FCSetDamping` | 设置力控阻尼 |
| `FCSetForceSpeedLimit` | 设置力控速度限制 |
| `FCOff` | 关闭力控 |
| `SetFCCollision` | 设置力控碰撞 |
| `FCCollisionSwitch` | 力控碰撞开关 |

### 2.13 Modbus 通信类

| 服务名 | 功能 |
|--------|------|
| `ModbusRTUCreate` | 创建 Modbus RTU 连接 |
| `ModbusCreate` | 创建 Modbus TCP 连接 |
| `ModbusClose` | 关闭 Modbus 连接 |
| `GetInBits` | 读取输入位 |
| `GetInRegs` | 读取输入寄存器 |
| `GetCoils` | 读取线圈 |
| `SetCoils` | 设置线圈 |
| `GetHoldRegs` | 读取保持寄存器 |
| `SetHoldRegs` | 设置保持寄存器 |

### 2.14 其他

| 服务名 | 功能 |
|--------|------|
| `GetErrorID` | 获取错误 ID 列表 |
| `GetError` | 获取错误信息 |
| `BrakeControl` | 抱闸控制 |
| `StartRTOffset` | 开始实时偏移 |
| `EndRTOffset` | 结束实时偏移 |

---

## 三、ros_control 与 MoveIt 支持状态

### 3.1 当前状态: ❌ 不支持

| 组件 | 状态 | 说明 |
|------|------|------|
| ros2_control Hardware Interface | ❌ 不支持 | `CRRobotRos2` 继承 `rclcpp::Node`，不实现 `hardware_interface::SystemInterface`，通过 TCP Socket 直连控制器 |
| FollowJointTrajectory Action | ❌ 不支持 | 虽然 `main.cpp` 拼出了 `/cr5_robot/joint_controller/follow_joint_trajectory` 字符串，但未创建 Action Server |
| MoveIt Simple Controller Manager | ❌ 不支持 | 缺少 FollowJointTrajectory Action 接口，MoveIt 无法发轨迹 |

### 3.2 moveit_test 配置说明

`moveit_test/config/` 下的配置文件为 **XTrainer 教学平台** (8 轴双臂) 准备的仿真配置，与当前 CR5 驱动不配套：

- `ros2_controllers.yaml`: 使用 `joint_trajectory_controller` 控制 `J1_1~J1_8, J2_1~J2_8`
- `x_trainer.ros2_control.xacro`: 使用 `mock_components/GenericSystem` (仅仿真)
- `moveit_controllers.yaml`: 配置了 `moveit_simple_controller_manager`

### 3.3 集成路线建议

要让 MoveIt Simple Controller Manager 工作，至少需要：

1. **实现 FollowJointTrajectory Action Server**：在 `CRRobotRos2` 中添加 `follow_joint_trajectory` action，接收 MoveIt 轨迹并使用 `ServoJ` 实时插补执行
2. **或实现 ros2_control Hardware Interface**：创建 `hardware_interface::SystemInterface` 子类，将 `read()`/`write()` 桥接到现有 TCP 通信层
3. **对齐关节命名**：确保 URDF/XACRO 的 `joint_names` 与 `param.json` 完全一致（已通过本次修改将 Arm1 对齐为 `J1_1~J1_6`，Arm2 对齐为 `J2_1~J2_6`）

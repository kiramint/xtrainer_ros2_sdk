# dobot_msgs_v4 — Dobot Nova2 ROS2 消息与接口定义

Dobot 官方 Nova2 V4.1 SDK 的 ROS2 消息包，定义了与机械臂 TCP 协议对应的 srv 接口和自定义 msg 类型。

该包为 `rosidl_interface_packages` 成员，编译生成 C++ / Python 头文件供 `cr_robot_ros2`（dobot_bringup_v4）使用。

## 消息类型

### Services (`srv/`)

所有服务均为 `dobot_msgs_v4/srv/<Name>`，请求和响应字段遵循 Dobot TCP Dashboard 协议。

#### 常用控制服务

| Service | 说明 |
|---------|------|
| `PowerOn` | 机械臂上电 |
| `EnableRobot` | 使能 |
| `DisableRobot` | 去使能 |
| `ClearError` | 清除错误状态 |
| `ResetRobot` | 复位 |
| `EmergencyStop` | 急停 |
| `RobotMode` | 查询运行模式 (1~9) |
| `StartDrag` | 进入拖拽示教 |
| `StopDrag` | 退出拖拽示教 |

#### 运动控制

| Service | 说明 |
|---------|------|
| `MovJ` | 关节空间点到点运动 |
| `MovL` | 直线运动 |
| `MovJIO` / `MovLIO` | 带 IO 控制的关节/直线运动 |
| `MoveJog` | 点动 |
| `ServoJ` | 关节空间伺服（实时跟随，单位**度**） |
| `ServoP` | 笛卡尔空间伺服 |
| `RelJointMovJ` | 关节空间相对运动 |
| `RelMovJTool` / `RelMovLTool` | 工具坐标系相对运动 |
| `RelMovJUser` / `RelMovLUser` | 用户坐标系相对运动 |

#### 运动控制参数

| Service | 说明 |
|---------|------|
| `SpeedFactor` | 设置全局速度比例 |
| `AccJ` / `AccL` | 设置关节/直线加速度 |
| `SetPayload` | 设置负载参数 |
| `SetCollisionLevel` | 设置碰撞检测等级 |
| `SetBackDistance` | 设置回退距离 |
| `Continue` / `Pause` / `Stop` | 运动流程控制 |

#### 坐标系统

| Service | 说明 |
|---------|------|
| `PositiveKin` | 正运动学 |
| `InverseKin` | 逆运动学 |
| `InverseSolution` | 逆解求多解 |
| `GetAngle` | 获取当前关节角度 |
| `GetPose` | 获取当前位姿 |
| `GetStartPose` | 获取启动位姿 |
| `SetTool` / `CalcTool` | 工具坐标系设置/计算 |
| `SetUser` / `CalcUser` | 用户坐标系设置/计算 |
| `StartRTOffset` / `EndRTOffset` | 实时偏移 |

#### IO 控制

| Service | 说明 |
|---------|------|
| `DI` / `DO` | 数字输入/输出 |
| `DIGroup` / `DOGroup` | 数字 IO 组读写 |
| `AI` / `AO` | 模拟输入/输出 |
| `GetInBits` / `GetInRegs` | 读取输入位/寄存器 |
| `SetHoldRegs` / `GetHoldRegs` | 读写保持寄存器 |
| `SetCoils` / `GetCoils` | 读写线圈 |
| `ModbusCreate` / `ModbusRTUCreate` / `ModbusClose` | Modbus 设备管理 |

#### 力控

| Service | 说明 |
|---------|------|
| `EnableFTSensor` | 启用力/力矩传感器 |
| `FCForceMode` | 力控模式 |
| `FCOff` | 关闭力控 |
| `FCSetForce` / `FCSetForceLimit` | 设定期望力/力限 |
| `FCSetStiffness` / `FCSetDamping` | 设定刚度/阻尼 |
| `FCSetMass` / `FCSetDeviation` | 设定质量/偏差 |
| `FCSetForceSpeedLimit` | 设定力控速度限制 |
| `FCCollisionSwitch` | 碰撞检测开关 |
| `GetForce` | 读取力传感器数据 |
| `SixForceHome` | 六维力传感器归零 |

#### 安全

| Service | 说明 |
|---------|------|
| `SetSafeSkin` / `EnableSafeSkin` | 安全皮肤设置/使能 |
| `SetSafeWallEnable` | 安全墙使能 |
| `SetWorkZoneEnable` | 工作区域限制使能 |
| `SetPostCollisionMode` | 碰撞后处理模式 |

#### 其他

| Service | 说明 |
|---------|------|
| `Arc` / `Circle` | 圆弧/整圆运动 |
| `RunScript` | 执行 Lua 脚本 |
| `StartPath` | 启动路径 |
| `CheckOddMovC/J/L` | 奇异点检查 |
| `ForceDriveMode` / `ForceDriveSpeed` | 力拖动模式/速度 |
| `GetError` / `GetErrorID` | 查询错误信息 |
| `GetCurrentCommandId` | 获取当前指令 ID |
| `SetOutputBool/Float/Int` | 设置输出变量 |
| `GetOutputBool/Float/Int` | 读取输出变量 |
| `GetInputBool/Float/Int` | 读取输入变量 |
| `RunTo` | 运行到指定点 |
| `SetTool485` / `SetToolMode` / `SetToolPower` | 工具 IO 配置 |

### Messages (`msg/`)

| Message | 字段 | 说明 |
|---------|------|------|
| `RobotStatus` | 机械臂运行状态信息 | |
| `ToolVectorActual` | 工具端实际位姿 | |

## 编译

```bash
colcon build --packages-select dobot_msgs_v4
source install/setup.bash
```

> [!NOTE]
> 该包仅供 `cr_robot_ros2` 依赖，不需要独立运行。

# xtrainer_gripper — 夹爪 ROS2 驱动包

基于 Feetech SMS/STS 系列舵机的 USB-485 夹爪独立驱动，与 `dobot_bringup`、`xtrainer_bridge` 完全解耦，通过话题通信。

单节点管理双臂（左右两个夹爪），左右夹爪使用独立串口和舵机 ID，控制方式完全相同。

## 一、话题

节点命名空间为 `/gripper`，左右夹爪话题通过子路径区分：

| 话题 | 消息类型 | 方向 | QoS | 功能 |
|------|----------|------|-----|------|
| `/gripper/left/command` | `Float32MultiArray` | 订阅 | 10 | 左夹爪控制 `[position, speed]` |
| `/gripper/left/state` | `Float32MultiArray` | 发布 | 10 | 左夹爪状态 `[position, load]` |
| `/gripper/left/status` | `String` | 发布 | 10 | 左夹爪状态字符串 |
| `/gripper/left/position` | `Int32` | 发布 | 10 | 左夹爪原始位置读数 |
| `/gripper/right/command` | `Float32MultiArray` | 订阅 | 10 | 右夹爪控制 |
| `/gripper/right/state` | `Float32MultiArray` | 发布 | 10 | 右夹爪状态 |
| `/gripper/right/status` | `String` | 发布 | 10 | 右夹爪状态字符串 |
| `/gripper/right/position` | `Int32` | 发布 | 10 | 右夹爪原始位置读数 |

### 1.1 `command` — 控制命令

```
data: [float32 position, float32 speed]
```

| 字段 | 范围 | 说明 |
|------|------|------|
| `position` | `0.0` ~ `1.0` | 目标位置。`0.0` = 完全张开，`1.0` = 完全闭合 |
| `speed` | `0.0` ~ `1.0` | 运动速度。`0.0` = 最慢，`1.0` = 全速。若缺省第二项则默认为 `1.0` |

**示例**:

```bash
# 左夹爪全速张开
ros2 topic pub /gripper/left/command std_msgs/Float32MultiArray \
  "{data: [0.0, 1.0]}"
# 右夹爪 30% 速度闭合
ros2 topic pub /gripper/right/command std_msgs/Float32MultiArray \
  "{data: [1.0, 0.3]}"
# 左夹爪半开
ros2 topic pub /gripper/left/command std_msgs/Float32MultiArray \
  "{data: [0.5, 0.5]}"
```

### 1.2 `state` — 实时状态

```
data: [float32 position, float32 load]
```

| 字段 | 范围 | 说明 |
|------|------|------|
| `position` | `0.0` ~ `1.0` | 当前位置（归一化），`-1.0` 表示读取失败 |
| `load` | `0.0` ~ `1.0` | 当前负载比例（舵机 PRESENT_LOAD 寄存器），`-1.0` 表示读取失败 |

> **负载说明**: 读取 SMS/STS 舵机地址 60-61 (`PRESENT_LOAD_L/H`)，原始范围 0~1000 对应额定力矩的 0%~100%，驱动归一化至 0~1。可用于碰撞检测和恒力夹取。

### 1.3 `position` — 原始位置读数

```
data: int32
```

| 字段 | 范围 | 说明 |
|------|------|------|
| `data` | `0` ~ `4095` | 电机原始位置值（未经归一化），`-1` 表示读取失败 |

### 1.4 `status` — 状态描述

| 状态字符串 | 含义 |
|-----------|------|
| `"OPENED"` | 已完全张开 (position < 0.05) |
| `"CLOSED"` | 已完全闭合 (position > 0.95) |
| `"MOVING"` | 运动中 (0.05 ≤ position ≤ 0.95) |
| `"ERROR"` | 通信异常（位置读取失败） |

---

## 二、参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `left_port` | string | `/dev/ttyUSB0` | 左夹爪 USB-485 串口 |
| `left_servo_id` | int | `21` | 左夹爪舵机 ID |
| `left_servo_min_pos` | int | `2500` | 左夹爪闭合位置（舵机内部值，对应 1.0） |
| `left_servo_max_pos` | int | `3800` | 左夹爪张开位置（舵机内部值，对应 0.0） |
| `right_port` | string | `/dev/ttyUSB1` | 右夹爪 USB-485 串口 |
| `right_servo_id` | int | `22` | 右夹爪舵机 ID |
| `right_servo_min_pos` | int | `2500` | 右夹爪闭合位置 |
| `right_servo_max_pos` | int | `3800` | 右夹爪张开位置 |
| `publish_rate` | float | `20.0` | 状态发布频率 (Hz) |
| `torque_limit` | int | `300` | 力矩限制 (0~1000) |

> **注意**: `servo_min_pos` 为闭合位置（舵机角度下限），`servo_max_pos` 为张开位置（舵机角度上限）。映射关系：`0.0`（张开）→ `servo_max_pos`，`1.0`（闭合）→ `servo_min_pos`。

**环境变量覆写**（通过 `gripper.launch.py` 启动时）:

| 环境变量 | 说明 |
|----------|------|
| `GRIPPER_NS` | 节点命名空间（默认 `gripper`） |
| `GRIPPER_LEFT_PORT` | 左夹爪串口 |
| `GRIPPER_LEFT_ID` | 左夹爪舵机 ID |
| `GRIPPER_LEFT_MIN` | 左夹爪最小位置 |
| `GRIPPER_LEFT_MAX` | 左夹爪最大位置 |
| `GRIPPER_RIGHT_PORT` | 右夹爪串口 |
| `GRIPPER_RIGHT_ID` | 右夹爪舵机 ID |
| `GRIPPER_RIGHT_MIN` | 右夹爪最小位置 |
| `GRIPPER_RIGHT_MAX` | 右夹爪最大位置 |

---

## 三、启动

### 3.1 编译

```bash
colcon build --packages-select xtrainer_gripper
source install/setup.bash
```

### 3.2 udev 延迟定时器（推荐，一次性配置）

USB-485 适配器默认 latency_timer 为 16ms，会显著增加通信延迟。运行以下脚本设置 udev 规则将其固定为 1ms：

```bash
sudo ./xtrainer_gripper/setup_udev.sh
```

配置后夹爪节点自动生效，无需 sudo。

### 3.3 Launch

```bash
# 默认参数（左右双夹爪）
ros2 launch xtrainer_gripper gripper.launch.py

# 通过环境变量指定参数
GRIPPER_LEFT_PORT=/dev/ttyUSB2 GRIPPER_RIGHT_PORT=/dev/ttyUSB3 \
  ros2 launch xtrainer_gripper gripper.launch.py
```

> 在 `start.launch.py` 中已包含夹爪节点，随双臂系统一同启动。

### 3.4 命令行测试

```bash
# 查看左夹爪状态
ros2 topic echo /gripper/left/state
ros2 topic echo /gripper/left/status
ros2 topic echo /gripper/left/position

# 控制左夹爪
ros2 topic pub /gripper/left/command std_msgs/Float32MultiArray "{data: [0.0, 1.0]}"  # 张开
ros2 topic pub /gripper/left/command std_msgs/Float32MultiArray "{data: [1.0, 1.0]}"  # 闭合

# 控制右夹爪
ros2 topic pub /gripper/right/command std_msgs/Float32MultiArray "{data: [0.0, 1.0]}"  # 张开
ros2 topic pub /gripper/right/command std_msgs/Float32MultiArray "{data: [1.0, 0.5]}"  # 半速闭合
```

---

## 四、Demo

### 4.1 开合循环 Demo

```bash
# 测试左夹爪（默认）
ros2 run xtrainer_gripper gripper_open_close_demo

# 测试右夹爪
ros2 run xtrainer_gripper gripper_open_close_demo --ros-args -p gripper_side:=right
```

执行 3 次张开→闭合→张开循环，每次等待状态到位，打印实时位置和负载。

### 4.2 恒力夹取 Demo

```bash
# 默认左夹爪
ros2 run xtrainer_gripper gripper_constant_force_demo

# 右夹爪
ros2 run xtrainer_gripper gripper_constant_force_demo --ros-args -p gripper_side:=right
```

工作流程:

1. 夹爪全开
2. 缓慢闭合 (0.3 速度)
3. 监测 `/gripper/left/state`（或 `right/state`）中的 `load` 值
4. 当 `load > 阈值(0.18)` → 停止闭合（物体已夹住）
5. 保持 5 秒并监控滑脱
6. 若负载下降超过 0.05 → 自动补夹半步
7. 滑脱超过 3 次 → 放弃并张开释放

**可调参数**（在 `gripper_constant_force_demo.py` 顶部修改）:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FORCE_THRESHOLD` | `0.18` | 负载阈值 (0~1) |
| `CLOSE_SPEED` | `0.3` | 夹取速度 (0~1) |
| `SLIP_DETECT_THRESHOLD` | `0.05` | 滑脱检测灵敏度 |
| `RETRY_STEP` | `0.08` | 补夹步长 |
| `HOLD_DURATION` | `5.0` | 保持时间 (秒) |

---

## 五、架构

```
┌─────────────────────────────────────────────────┐
│                用户 / MoveIt / Demo              │
└────────┬────────────────────────────┬───────────┘
         │  left/command              │  right/command
         │  left/state, left/status   │  right/state, right/status
         │  left/position             │  right/position
         ▼                            ▼
┌─────────────────────────────────────────────────┐
│              gripper_node (单节点)                │
│              namespace: /gripper                 │
├─────────────────────┬───────────────────────────┤
│  _GripperDriver      │  _GripperDriver           │
│  side="left"         │  side="right"             │
│  port=/dev/ttyUSB0   │  port=/dev/ttyUSB1        │
│  servo_id=21         │  servo_id=22              │
├─────────────────────┼───────────────────────────┤
│         scservo_sdk/ (Feetech SMS/STS 协议)       │
└─────────────────────┴───────────────────────────┘
         │                            │
         │ USB-485                    │ USB-485
         ▼                            ▼
┌─────────────────┐          ┌─────────────────┐
│ Feetech 舵机     │          │ Feetech 舵机     │
│ ID=21 (左)      │          │ ID=22 (右)      │
└─────────────────┘          └─────────────────┘
```

- **单节点管理双臂**: 一个进程同时控制左右两个夹爪，各自独立串口和舵机 ID
- **完全解耦**: 不依赖 `dobot_bringup`、`xtrainer_bridge`、CRobotRos2
- **自有 launch 文件**: 可独立启动
- **自有 SDK**: `scservo_sdk/` 内嵌，无需外部依赖

---

## 六、硬件接口

| 项目 | 说明 |
|------|------|
| 舵机型号 | Feetech SMS / STS 系列 |
| 通信接口 | USB-485 转换器 ×2 |
| 协议 | SMS/STS 二进制协议 (`protocol_end=0`) |
| 波特率 | 1,000,000 bps |
| 供电 | 舵机独立供电，USB-485 仅信号线 |
| 左夹爪 | `/dev/ttyUSB0`, 舵机 ID=21 |
| 右夹爪 | `/dev/ttyUSB1`, 舵机 ID=22 |

# xtrainer_gripper — 夹爪 ROS2 驱动包

基于 Feetech SMS/STS 系列舵机的 USB-485 夹爪独立驱动，与 `dobot_bringup`、`xtrainer_bridge` 完全解耦，通过话题通信。

## 一、话题

| 话题 | 消息类型 | 方向 | QoS | 功能 |
|------|----------|------|-----|------|
| `/gripper/command` | `std_msgs/Float32MultiArray` | 订阅 | 10 | 控制命令 `[position, speed]` |
| `/gripper/state` | `std_msgs/Float32MultiArray` | 发布 | 10 | 实时状态 `[position, load]` |
| `/gripper/status` | `std_msgs/String` | 发布 | 10 | 状态字符串 |

### 1.1 `/gripper/command` — 控制命令

```
data: [float32 position, float32 speed]
```

| 字段 | 范围 | 说明 |
|------|------|------|
| `position` | `0.0` ~ `1.0` | 目标位置。`0.0` = 完全张开，`1.0` = 完全闭合 |
| `speed` | `0.0` ~ `1.0` | 运动速度。`0.0` = 最慢，`1.0` = 全速。若缺省第二项则默认为 `1.0` |

**示例**:

```bash
ros2 topic pub /gripper/command std_msgs/Float32MultiArray \
  "{data: [0.0, 1.0]}"   # 全速张开
ros2 topic pub /gripper/command std_msgs/Float32MultiArray \
  "{data: [1.0, 0.3]}"   # 30%速度闭合
ros2 topic pub /gripper/command std_msgs/Float32MultiArray \
  "{data: [0.5, 0.5]}"   # 半开
```

### 1.2 `/gripper/state` — 实时状态

```
data: [float32 position, float32 load]
```

| 字段 | 范围 | 说明 |
|------|------|------|
| `position` | `0.0` ~ `1.0` | 当前位置 (归一化)，`-1.0` 表示读取失败 |
| `load` | `0.0` ~ `1.0` | 当前负载比例 (舵机 PRESENT_LOAD 寄存器)，`-1.0` 表示读取失败 |

> **负载说明**: 读取 SMS/STS 舵机地址 60-61 (`PRESENT_LOAD_L/H`)，原始范围 0~1000 对应额定力矩的 0%~100%，驱动归一化至 0~1。可用于碰撞检测和恒力夹取。

### 1.3 `/gripper/status` — 状态描述

| 状态字符串 | 含义 |
|-----------|------|
| `"OPENED"` | 已完全张开 (position < 0.05) |
| `"CLOSED"` | 已完全闭合 (position > 0.95) |
| `"MOVING"` | 运动中 (0.05 ≤ position ≤ 0.95) |
| `"ERROR"` | 通信异常 (位置读取失败) |

---

## 二、参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyUSB0` | USB-485 串口设备路径 |
| `servo_id` | int | `1` | 舵机 ID |
| `servo_min_pos` | int | `2048` | 舵机内部最小位置（对应 0.0 开） |
| `servo_max_pos` | int | `3998` | 舵机内部最大位置（对应 1.0 关） |
| `publish_rate` | float | `20.0` | 状态发布频率 (Hz) |

**环境变量覆写**（launch 时）:

| 环境变量 | 说明 |
|----------|------|
| `GRIPPER_PORT` | 串口设备路径 |
| `GRIPPER_SERVO_ID` | 舵机 ID |
| `GRIPPER_SERVO_MIN` | 舵机最小位置 |
| `GRIPPER_SERVO_MAX` | 舵机最大位置 |

---

## 三、启动

### 3.1 编译

```bash
colcon build --packages-select xtrainer_gripper
source install/setup.bash
```

### 3.2 Launch

```bash
# 默认参数
ros2 launch xtrainer_gripper gripper.launch.py

# 指定串口和舵机参数
GRIPPER_PORT=/dev/ttyUSB1 GRIPPER_SERVO_ID=2 \
  ros2 launch xtrainer_gripper gripper.launch.py
```

### 3.3 命令行测试

```bash
# 查看状态
ros2 topic echo /gripper/state
ros2 topic echo /gripper/status

# 控制
ros2 topic pub /gripper/command std_msgs/Float32MultiArray "{data: [0.0, 1.0]}"  # 张开
ros2 topic pub /gripper/command std_msgs/Float32MultiArray "{data: [1.0, 1.0]}"  # 闭合
```

---

## 四、Demo

### 4.1 开合循环 Demo

```bash
python3 xtrainer_gripper/demo/gripper_open_close_demo.py
```

执行 3 次张开→闭合→张开循环，每次等待状态到位，打印实时位置和负载。

### 4.2 恒力夹取 Demo

```bash
python3 xtrainer_gripper/demo/gripper_constant_force_demo.py
```

工作流程:

1. 夹爪全开
2. 缓慢闭合 (0.3 速度)
3. 监测 `/gripper/state` 中的 `load` 值
4. 当 `load > 阈值(0.18)` → 停止闭合（物体已夹住）
5. 保持 5 秒并监控滑脱
6. 若负载下降超过 0.05 → 自动补夹半步
7. 滑脱超过 3 次 → 放弃并张开释放

**可调参数** (在 `gripper_constant_force_demo.py` 顶部修改):

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
┌─────────────────────────────────────────────┐
│                 用户 / MoveIt                 │
│                  Demo 脚本                    │
└───────────┬─────────────┬───────────────────┘
            │ 发布         │ 订阅
            ▼              ▲
┌───────────────────────────────────────────────┐
│          /gripper/command                     │
│          /gripper/state                       │
│          /gripper/status                      │
└───────────────────┬───────────────────────────┘
                    │
            ┌───────▼───────┐
            │ gripper_node  │  ← xtrainer_gripper 包
            │ (GripperNode) │
            ├───────────────┤
            │ scservo_sdk/  │  ← Feetech SDK
            │  sms_sts      │     (SMS/STS 协议)
            │  port_handler │
            └───────┬───────┘
                    │ USB-485
            ┌───────▼───────┐
            │ Feetech 舵机   │
            │ (SMS/STS系列)  │
            └───────────────┘
```

- **完全解耦**: 不依赖 `dobot_bringup`、`xtrainer_bridge`、CRobotRos2
- **自有 launch 文件**: 可独立启动
- **自有 SDK**: `scservo_sdk/` 内嵌，无需外部依赖

---

## 六、硬件接口

| 项目 | 说明 |
|------|------|
| 舵机型号 | Feetech SMS / STS 系列 |
| 通信接口 | USB-485 转换器 |
| 协议 | SMS/STS 二进制协议 (`protocol_end=0`) |
| 波特率 | 1,000,000 bps |
| 供电 | 舵机独立供电，USB-485 仅信号线 |

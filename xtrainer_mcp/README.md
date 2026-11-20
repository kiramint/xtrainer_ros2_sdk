# xtrainer_mcp

给 agent 用的 XTrainer 接口。这个包不启动硬件，也不在进程里放一份 MoveIt。它订阅已经在跑的 ROS 图，规划、逆解和自碰撞都交给 `move_group`。

先拉起机器人和 MoveIt，再由 agent 另开一个进程启动 MCP。MCP 走 stdio，stdout 只能给协议，所以不能放进 `ros2 launch`。

## 启动

```bash
# 驱动、相机、夹爪、手眼 TF、move_group、RViz
ros2 launch xtrainer_mcp bringup.launch.py

# 不要 RViz
ros2 launch xtrainer_mcp bringup.launch.py use_rviz:=false
```

抱闸仍由 `cr_robot_ros2_node` 负责：连上控制器后解锁，进程退出时锁定。

不要同时再跑下面两个：

- `ros2 launch xtrainer_moveit demo.launch.py` 会再起一个 `robot_state_publisher`
- `xtrainer_task` 里会执行轨迹的脚本。执行器只能有 `move_group` 这一份

`demo.launch.py` 仍然可以单独配合 `start.launch.py` 使用。和 `bringup.launch.py` 二选一。

## Agent 配置

Python 包 `mcp`（2.x，入口是 `mcp.server.mcpserver.MCPServer`）放在本包的 `.vendor` 里，进程启动时会把它加进 `sys.path`。不要装进系统 Python：Ubuntu 24.04 把 `/usr/bin/python3` 标成外部托管，而 `ros2 run` 用的就是这个解释器。

还没有 `.vendor` 时执行一次：

```bash
python3 -m pip install --target /opt/Project/dobot_ros2_sdk/xtrainer_mcp/.vendor --break-system-packages 'mcp>=2,<3'
```

`--target` 只往 `.vendor` 里写。`--break-system-packages` 只是越过外部托管检查，不会改 `/usr`。

ROS 和本工作空间要先 source，并且 `ROS_DOMAIN_ID` 与 bringup 相同。

```json
{
  "mcpServers": {
    "xtrainer": {
      "command": "bash",
      "args": [
        "-lc",
        "source /opt/ros/jazzy/setup.bash && source /opt/Project/dobot_ros2_sdk/install/setup.bash && ros2 run xtrainer_mcp xtrainer_mcp"
      ]
    }
  }
}
```

日志在 stderr。

## 话题和服务

相机名是 `camera_top`、`camera_left`、`camera_right`。

| 名字 | 作用 |
| --- | --- |
| `/joint_states` | 12 个驱动关节，弧度 |
| `/camera/<name>/color/image_raw` | 彩色图 |
| `/camera/<name>/aligned_depth_to_color/image_raw` | 对齐深度，uint16 毫米 |
| `/camera/<name>/color/camera_info` | 内参 K |
| `/tf`、`/tf_static` | 手眼标定和机器人模型 |
| `/display_planned_path` | 规划结果，RViz 能看到 |
| `/plan_kinematic_path` | 只规划 |
| `/execute_trajectory` | 执行缓存的完整轨迹 |
| `/compute_fk`、`/compute_ik` | 正逆运动学 |
| `/check_state_validity` | 整机自碰撞，不含点云 |
| `/Arm1_controller/follow_joint_trajectory`、`/Arm2_controller/follow_joint_trajectory` | move_group 实际下发的动作 |
| `/ArmX/dobot_bringup_ros2/srv/ServoJ` | 直接关节伺服，接口里用弧度，发出去是度 |
| `/gripper/left/*`、`/gripper/right/*` | 夹爪 |
| `/ArmX/dobot_bringup_ros2/srv/EnableRobot`、`DisableRobot`、`ClearError`、`RobotMode` | 抱闸和报警 |

深度光学坐标系是 `camera_<name>_color_optical_frame`。位姿默认在 `base_link`。关节对外都是弧度。

## 工具

| 工具 | 作用 |
| --- | --- |
| `get_camera_frame` | 最新彩色 JPEG 和 16 位深度 PNG。没有画面立刻报错 |
| `project_pixel` | 像素 → 光学坐标 → `base_link`。深度用窗口中值，变换用该帧时间戳 |
| `transform_optical_point` | 光学坐标系里的点，用最新 TF 变到 `base_link` |
| `get_tcp_pose` | TCP 或 tip 的世界位姿 |
| `get_status` | 双臂模式、关节、TCP、夹爪、已缓存的计划号 |
| `plan_joints` / `plan_pose` / `plan_named` | 只规划。JSON 超过 200 点会抽稀，并写明原始点数 |
| `execute_plan` | 执行完整轨迹。起点差超过 0.05 rad、自碰撞或正在动时拒绝 |
| `check_self_collision` / `check_joints` / `check_plan` | 整机自碰撞和接触点 |
| `fk` / `ik` | 优先走 move_group。FK 服务不在时用同一份 URDF |
| `servo_joints` | 最多 60 点，dt 0.02–0.5 秒，单步 ≤ 0.15 rad，总时长 ≤ 5 秒 |
| `gripper_open` / `gripper_close` / `gripper_set_torque` / `gripper_get_state` | 夹爪。力矩 0–1000 |
| `clear_error` / `unlock_brakes` / `lock_brakes` / `get_robot_mode` | 报警和抱闸。解锁会等到模式 5/6/7/8/10 |

`arm` 取 `Arm1` 或 `Arm2`。抱闸和报警还可以取 `both`。夹爪 `side` 取 `left`、`right` 或 `both`。`link` 取 `tcp`（默认，`L1_gripper_tcp` / `L2_gripper_tcp`）或 `tip`。

规划器：`ompl`（RRTConnect）、`pilz_ptp`、`pilz_lin`。速度比例 0.3，加速度比例 0.1。

执行、ServoJ、夹爪和抱闸同一时间只能有一个。读状态、规划、FK、IK 和碰撞检查可以并行。`execute_plan` 会一直等到结束，这一版没有取消。

命名姿态来自 SRDF：`Home1` 属于 Arm1，`Home2` 属于 Arm2，都是各关节 0。

# xtrainer_description — XTrainer URDF 与 3D 模型

XTrainer 双臂机器人的 URDF 描述文件与 Mesh 3D 模型，供 `robot_state_publisher` 发布 TF 变换和 MoveIt 运动规划使用。

## 文件结构

| 路径 | 说明 |
|------|------|
| `urdf/x_trainer.urdf` | 整机 URDF（不含 xacro，直接使用） |
| `meshes/` | OBJ + MTL 3D 模型文件 |

## Frame 命名约定

### 基座

- `base_link` — 机器人根坐标系（通过 `root_joint` 固定于 `world`）

### Arm1（左臂）

| 关节 | Link | 类型 |
|------|------|------|
| J1_1 ~ J1_6 | L1_1 ~ L1_6 | 6 个旋转关节 |
| J1_7, J1_8 | L1_7, L1_8 | 2 个 prismatic 关节（夹爪手指） |
| J1_gripper_tcp (fixed) | L1_gripper_tcp | 末端 TCP |

- **末端 effector frame**: `L1_gripper_tcp`

### Arm2（右臂）

| 关节 | Link | 类型 |
|------|------|------|
| J2_1 ~ J2_6 | L2_1 ~ L2_6 | 6 个旋转关节 |
| J2_7, J2_8 | L2_7, L2_8 | 2 个 prismatic 关节（夹爪手指） |
| J2_gripper_tcp (fixed) | L2_gripper_tcp | 末端 TCP |

- **末端 effector frame**: `L2_gripper_tcp`

> Arm3 (J3_*) 和 Arm4 (J4_*) 也存在于 URDF 中，但目前未使用。

### 关节角度限制

| 关节序号 | 1 | 2 | 3 | 4 | 5 | 6 |
|---------|---|---|---|---|---|---|
| 角度范围 (rad) | ±6.28 | ±3.14 | ±2.79 | ±6.28 | ±6.28 | ±6.28 |
| (度) | ±360 | ±180 | ±160 | ±360 | ±360 | ±360 |

> URDF 中 velocity/effort 全部设为 0（不限制），关节级限速由 `moveit_test` 的 `joint_limits.yaml` 管理。

## 使用

该包不包含可执行节点，仅提供 URDF 和 Mesh 文件。被以下包引用：

- `cr_robot_ros2/xtrainer.launch.py` — `robot_state_publisher` 加载 URDF 发布 TF
- `moveit_test` — MoveIt 的 URDF/SRDF 配置入口 (`x_trainer.urdf.xacro`)
- `xtrainer_task` — MoveItPy 加载机器人模型

## 加载方式

```python
from ament_index_python.packages import get_package_share_directory
import os

urdf_path = os.path.join(
    get_package_share_directory('xtrainer_description'),
    'urdf', 'x_trainer.urdf'
)
with open(urdf_path, 'r') as f:
    robot_description = f.read()
```

## 编译

```bash
colcon build --packages-select xtrainer_description
source install/setup.bash
```

## 硬件验证

启动 MoveIt 后手动移动机械臂，对照 URDF 模型与实物确认安装准确性。错误的安装会导致运动规划结果与现实不一致，且碰撞检测失效。

> CAD 文件来源：Dobot 官网 [轻量版3D模型-5.28.step](https://www.dobot.cn/service/download-center?keyword=&type-95%5B%5D=112)

# 双臂同时启动

import json
import os

import launch_ros.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
cur_json_path = os.path.join(cur_config_path, 'param.json')

# 读取 JSON 文件
with open(cur_json_path, 'r') as file:
    json_data = json.load(file)

robot_number = json_data["robot_number"]
node_info = json_data["node_info"]

# 左臂 (Arm1) 配置 — node_info[0]
arm1_info = node_info[0]
arm1_ip = os.getenv("ARM1_IP", arm1_info["ip_address"])

# 右臂 (Arm2) 配置 — node_info[1]
arm2_info = node_info[1]
arm2_ip = os.getenv("ARM2_IP", arm2_info["ip_address"])

# URDF 路径
xtrainer_description_pkg = get_package_share_directory('xtrainer_description')
urdf_path = os.path.join(xtrainer_description_pkg, 'urdf', 'x_trainer.urdf')

# 如果只有一个 URDF 文件（不是 xacro），直接读取；否则解析 xacro
if urdf_path.endswith('.urdf'):
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
else:
    robot_description = xacro.process_file(urdf_path).toxml()


def generate_launch_description():
    # ── robot_state_publisher: 根据 URDF + /joint_states 发布 /tf 和 /tf_static ──
    rsp_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': '',           # 不加前缀，link 名与 URDF 一致
            'ignore_timestamp': False,    # 使用 joint_states 的时间戳
        }],
    )

    # 左臂节点 — namespace Arm1
    arm1_node = launch_ros.actions.Node(
        package='cr_robot_ros2',
        executable='cr_robot_ros2_node',
        name=arm1_info["robot_node_name"],
        namespace='Arm1',
        output='screen',
        parameters=[{
            "robot_ip_address": arm1_ip,
            "robot_type": arm1_info["robot_type"],
            "trajectory_duration": arm1_info["trajectory_duration"],
            "robot_node_name": arm1_info["robot_node_name"],
            "robot_number": 1,
            "joint_names": arm1_info["joint_names"],
            "JointStatePublishRate": 50.0,
        }],
    )

    # 右臂节点 — namespace Arm2
    arm2_node = launch_ros.actions.Node(
        package='cr_robot_ros2',
        executable='cr_robot_ros2_node',
        name=arm2_info["robot_node_name"],
        namespace='Arm2',
        output='screen',
        parameters=[{
            "robot_ip_address": arm2_ip,
            "robot_type": arm2_info["robot_type"],
            "trajectory_duration": arm2_info["trajectory_duration"],
            "robot_node_name": arm2_info["robot_node_name"],
            "robot_number": 1,
            "joint_names": arm2_info["joint_names"],
            "JointStatePublishRate": 50.0,
        }],
    )

    # 桥接节点 — 合并 joint_states + FollowJointTrajectory Action Server
    bridge_node = launch_ros.actions.Node(
        package='xtrainer_bridge',
        executable='xtrainer_bridge_node',
        name='xtrainer_bridge',
        output='screen',
        parameters=[{
            "arm1_joint_names": arm1_info["joint_names"],
            "arm2_joint_names": arm2_info["joint_names"],
            "arm1_servoj_service": "/Arm1/dobot_bringup_ros2/srv/ServoJ",
            "arm2_servoj_service": "/Arm2/dobot_bringup_ros2/srv/ServoJ",
            "arm1_joint_state_topic": "/Arm1/joint_states_robot",
            "arm2_joint_state_topic": "/Arm2/joint_states_robot",
        }],
    )

    return LaunchDescription([rsp_node, arm1_node, arm2_node, bridge_node])

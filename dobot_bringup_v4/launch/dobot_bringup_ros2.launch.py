import json
import os

import launch_ros.actions
from launch import LaunchDescription

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
cur_json_path = os.path.join(cur_config_path, 'param.json')

# 读取 JSON 文件
with open(cur_json_path, 'r') as file:
    json_data = json.load(file)

node_info = json_data["node_info"]

# 默认取第一个机械臂配置
robot_index = 0
trajectory_duration = node_info[robot_index]["trajectory_duration"]
robot_node_name = node_info[robot_index]["robot_node_name"]
joint_names = node_info[robot_index]["joint_names"]
ip_address = os.getenv("IP_address")
              
robot_type = os.getenv("DOBOT_TYPE")


dobot_ros2_params = [
    {"robot_ip_address": ip_address},
    {"robot_type": robot_type},
    {"trajectory_duration": trajectory_duration},
    {"robot_node_name": robot_node_name},
    {"joint_names": joint_names},
]


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='cr_robot_ros2',
            executable='cr_robot_ros2_node',
            name=robot_node_name,
            output='screen',
            parameters=dobot_ros2_params
            # respawn=True
        ),
    ])

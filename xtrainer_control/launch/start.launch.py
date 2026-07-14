import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ================================================================
    # XTrainer 机械臂驱动
    # ================================================================
    xtrainer_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("dobot_bringup_v4"),
            "launch",
            "xtrainer.launch.py",
        )
    )

    # ================================================================
    # 夹爪节点
    # ================================================================
    gripper_1 = Node(
        package="xtrainer_gripper",
        executable="gripper_node",
        name="gripper_node",
        output="screen",
        parameters=[
            {"port": "/dev/ttyUSB0"},
            {"servo_id": 1},
            {"servo_min_pos": 2048},
            {"servo_max_pos": 3998},
            {"publish_rate": 20.0},
        ],
    )

    gripper_2 = Node(
        package="xtrainer_gripper",
        executable="gripper_node",
        name="gripper_node",
        output="screen",
        parameters=[
            {"port": "/dev/ttyUSB1"},
            {"servo_id": 1},
            {"servo_min_pos": 2048},
            {"servo_max_pos": 3998},
            {"publish_rate": 20.0},
        ],
    )

    # ================================================================
    # RealSense 相机
    # ================================================================
    realsense_camera_top = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        ),
        launch_arguments={
            # --- SN 绑定 ---
            "serial_no": "",
            # --- 相机命名与命名空间 ---
            "camera_name": "camera_top",
            "camera_namespace": "camera",
            # --- 启用深度与彩色流 ---
            "enable_depth": "true",
            "enable_color": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            # --- 点云 ---
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "false",
            "pointcloud.allow_no_texture_points": "true",
            # --- 深度对齐到彩色 (生成对齐的深度图 & 彩色点云) ---
            "align_depth.enable": "true",
            # --- 深度着色 (将深度图转为彩色便于可视化) ---
            "colorizer.enable": "true",
            # --- TF ---
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
            # --- 输出 ---
            "output": "screen",
            "log_level": "info",
        }.items(),
    )

    realsense_camera_left = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        ),
        launch_arguments={
            # --- SN 绑定 ---
            "serial_no": "",
            # --- 相机命名与命名空间 ---
            "camera_name": "camera_left",
            "camera_namespace": "camera",
            # --- 启用深度与彩色流 ---
            "enable_depth": "true",
            "enable_color": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            # --- 点云 ---
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "false",
            "pointcloud.allow_no_texture_points": "true",
            # --- 深度对齐到彩色 (生成对齐的深度图 & 彩色点云) ---
            "align_depth.enable": "true",
            # --- 深度着色 (将深度图转为彩色便于可视化) ---
            "colorizer.enable": "true",
            # --- TF ---
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
            # --- 输出 ---
            "output": "screen",
            "log_level": "info",
        }.items(),
    )

    realsense_camera_right = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        ),
        launch_arguments={
            # --- SN 绑定 ---
            "serial_no": "",
            # --- 相机命名与命名空间 ---
            "camera_name": "camera_right",
            "camera_namespace": "camera",
            # --- 启用深度与彩色流 ---
            "enable_depth": "true",
            "enable_color": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            # --- 点云 ---
            "pointcloud.enable": "true",
            "pointcloud.ordered_pc": "false",
            "pointcloud.allow_no_texture_points": "true",
            # --- 深度对齐到彩色 (生成对齐的深度图 & 彩色点云) ---
            "align_depth.enable": "true",
            # --- 深度着色 (将深度图转为彩色便于可视化) ---
            "colorizer.enable": "true",
            # --- TF ---
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
            # --- 输出 ---
            "output": "screen",
            "log_level": "info",
        }.items(),
    )

    ld = []

    # ================================================================
    # Calibrate Data
    # ================================================================
    if os.path.exists(
        os.path.expanduser(
            "~/.ros2/easy_handeye2/calibrations/top_cam_cal.calib"
        )
    ):
        ld.append(
            # publish_tf_left
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("easy_handeye2"), "launch", "publish.launch.py"
                ),
                launch_arguments={
                    "name": "top_cam_cal"
                }.items()
            ),
        )
    if os.path.exists(
        os.path.expanduser(
            "~/.ros2/easy_handeye2/calibrations/left_cam_cal.calib"
        )
    ):
        ld.append(
            # publish_tf_left
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("easy_handeye2"), "launch", "publish.launch.py"
                ),
                launch_arguments={
                    "name": "left_cam_cal"
                }.items()
            ),
        )
    if os.path.exists(
        os.path.expanduser(
            "~/.ros2/easy_handeye2/calibrations/right_cam_cal.calib"
        )
    ):
        ld.append(
            # publish_tf_left
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("easy_handeye2"), "launch", "publish.launch.py"
                ),
                launch_arguments={
                    "name": "right_cam_cal"
                }.items()
            ),
        )

    
    # ================================================================
    # LaunchDescription
    # ================================================================

    ld.append([
            xtrainer_driver,
            gripper_1,
            gripper_2,
            realsense_camera_top,
            realsense_camera_left,
            realsense_camera_right,
        ])

    return LaunchDescription(
        ld
    )

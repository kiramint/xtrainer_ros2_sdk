import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

aruco_single_params = {
        "image_is_rectified": True,
        "marker_id": 99,
        "marker_size": 0.078,
        "reference_frame": "camera_left_color_optical_frame",
        "camera_frame": "camera_left_color_optical_frame",
        "marker_frame": "aruco_marker_left",
        "corner_refinement": "LINES",
    }
aruco_camera_info_topic_name = "/camera/camera_left/color/camera_info"
aruco_image_topic_name = "/camera/camera_left/color/image_raw"

easy_handeye_launch_path = os.path.join(
    get_package_share_directory("easy_handeye2"), "launch", "calibrate.launch.py"
)

def generate_launch_description():
    # ================================================================
    # RealSense 左手相机 (眼在手上)
    # ================================================================
    realsense_camera_left = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        ),
        launch_arguments={
            # --- SN 绑定 ---
            "serial_no": "'412622272023'",
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
            "colorizer.enable": "false",
            # --- TF ---
            "publish_tf": "true",
            "tf_publish_rate": "0.0",
            # --- 输出 ---
            "output": "screen",
            "log_level": "info",
        }.items(),
    )

    # ================================================================
    # XTrainer 机械臂驱动
    # ================================================================
    # xtrainer_driver = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("dobot_bringup_v4"),
    #         "launch",
    #         "xtrainer.launch.py",
    #     )
    # )

    return LaunchDescription([
        # xtrainer_driver,
        realsense_camera_left,
        Node(
            package="aruco_ros",
            executable="single",
            parameters=[aruco_single_params],
            remappings=[
                ("/camera_info", aruco_camera_info_topic_name),
                ("/image", aruco_image_topic_name),
            ],
        ),
        # easy_handeye2_calibrate left camera (eye_in_hand)
        IncludeLaunchDescription(
            easy_handeye_launch_path,
            launch_arguments={
                "calibration_type": "eye_in_hand",
                "name": "left_cam_cal",
                "robot_base_frame": "base_link",
                "robot_effector_frame": "L1_6",
                "tracking_base_frame": "camera_left_color_optical_frame",
                "tracking_marker_frame": "aruco_marker_left",
            }.items(),
        ),
    ])
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


def _make_pc_node(camera_name: str) -> Node:
    """为指定相机创建 depth_image_proc/point_cloud_xyzrgb 节点。

    用 /aligned_depth_to_color/image_raw (realsense 已施加 decimation/spatial/
    temporal/hole_filling 四重滤波并对齐到 color 像素网格) + color + color/camera_info
    生成有序彩色点云，发布到与 realsense 原路径一致的 /camera/<name>/depth/color/points。

    与 realsense 自带点云的区别:
      - frame_id: camera_<name>_color_optical_frame (而非 depth_optical_frame)
      - H×W: color 分辨率 (而非 decimation 后的 depth 分辨率)
      - SAM 在 color 域的 mask 可以 1:1 直接作用到点云网格
    """
    return Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name=f"{camera_name}_point_cloud_xyzrgb",
        namespace=f"/camera/{camera_name}",
        remappings=[
            ("depth_registered/image_rect", "aligned_depth_to_color/image_raw"),
            ("rgb/image_rect_color", "color/image_raw"),
            ("rgb/camera_info", "color/camera_info"),
            ("points", "depth/color/points"),
        ],
        output="screen",
    )


def generate_launch_description():
    # ================================================================
    # XTrainer 机械臂驱动
    # ================================================================
    xtrainer_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("cr_robot_ros2"),
            "launch",
            "xtrainer.launch.py",
        )
    )

    # ================================================================
    # 夹爪节点 (单节点管理双臂，话题: /gripper/left/xxx, /gripper/right/xxx)
    # ================================================================
    gripper_node = Node(
        package="xtrainer_gripper",
        executable="gripper_node",
        name="gripper",
        namespace="gripper",
        output="screen",
        parameters=[
            {"left_port": "/dev/ttyUSB0"},
            {"left_servo_id": 21},
            {"left_servo_min_pos": 1981},
            {"left_servo_max_pos": 3069},
            {"right_port": "/dev/ttyUSB1"},
            {"right_servo_id": 22},
            {"right_servo_min_pos": 1981},
            {"right_servo_max_pos": 3069},
            {"publish_rate": 20.0},
            {"torque_limit": 300},
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
            "serial_no": "'412622270884'",
            # --- 相机命名与命名空间 ---
            "camera_name": "camera_top",
            "camera_namespace": "camera",
            # --- 启用深度与彩色流 ---
            "enable_depth": "true",
            "enable_color": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            # --- 点云 ---
            # 关闭 realsense 自带点云，改用下方 depth_image_proc 生成
            # (realsense 的 /depth/color/points 在 depth 坐标系，与 SAM mask 错位)
            "pointcloud.enable": "false",
            "pointcloud.ordered_pc": "true",
            "pointcloud.allow_no_texture_points": "false",
            "decimation_filter.enable":"true",
            "spatial_filter.enable":"true",
            "temporal_filter.enable":"true",
            "hole_filling_filter.enable":"true",
            # --- 深度对齐到彩色 (生成对齐的深度图 & 彩色点云) ---
            "align_depth.enable": "true",
            # --- 深度着色 (将深度图转为彩色便于可视化) ---
            "colorizer.enable": "false",
            # --- 彩色流分辨率 ---
            "rgb_camera.color_profile": "1280x720x15",
            "depth_module.depth_profile": "1280x720x15",
            "depth_module.color_profile": "1280x720x15",
            "depth_module.infra_profile": "1280x720x15",
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
            # 关闭 realsense 自带点云，改用下方 depth_image_proc 生成
            # (realsense 的 /depth/color/points 在 depth 坐标系，与 SAM mask 错位)
            "pointcloud.enable": "false",
            "pointcloud.ordered_pc": "true",
            "pointcloud.allow_no_texture_points": "false",
            "decimation_filter.enable":"true",
            "spatial_filter.enable":"true",
            "temporal_filter.enable":"true",
            "hole_filling_filter.enable":"true",
            # --- 深度对齐到彩色 (生成对齐的深度图 & 彩色点云) ---
            "align_depth.enable": "true",
            # --- 深度着色 (将深度图转为彩色便于可视化) ---
            "colorizer.enable": "false",
            # --- 彩色流分辨率 ---
            # "rgb_camera.color_profile": "1280x720x15",
            # "depth_module.depth_profile": "1280x720x15",
            # "depth_module.color_profile": "1280x720x15",
            # "depth_module.infra_profile": "1280x720x15",
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
            "serial_no": "'412622270837'",
            # --- 相机命名与命名空间 ---
            "camera_name": "camera_right",
            "camera_namespace": "camera",
            # --- 启用深度与彩色流 ---
            "enable_depth": "true",
            "enable_color": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            # --- 点云 ---
            # 关闭 realsense 自带点云，改用下方 depth_image_proc 生成
            # (realsense 的 /depth/color/points 在 depth 坐标系，与 SAM mask 错位)
            "pointcloud.enable": "false",
            "pointcloud.ordered_pc": "true",
            "pointcloud.allow_no_texture_points": "false",
            "decimation_filter.enable":"true",
            "spatial_filter.enable":"true",
            "temporal_filter.enable":"true",
            "hole_filling_filter.enable":"true",
            # --- 深度对齐到彩色 (生成对齐的深度图 & 彩色点云) ---
            "align_depth.enable": "true",
            # --- 深度着色 (将深度图转为彩色便于可视化) ---
            "colorizer.enable": "false",
             # --- 彩色流分辨率 ---
            # "rgb_camera.color_profile": "1280x720x15",
            # "depth_module.depth_profile": "1280x720x15",
            # "depth_module.color_profile": "1280x720x15",
            # "depth_module.infra_profile": "1280x720x15",
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
    # 使能机械臂 (延迟启动，等待驱动节点就绪)
    # ================================================================
    enable_arms = TimerAction(
        period=3.0,  # 等待 3s 确保驱动节点 TCP 连接建立
        actions=[
            Node(
                package="xtrainer_control",
                executable="enable_arms",
                name="enable_arms_startup",
                output="screen",
                parameters=[{
                    "namespaces": ["Arm1", "Arm2"],
                }],
            ),
        ],
    )

    # ================================================================
    # LaunchDescription
    # ================================================================

    ld.extend([
            xtrainer_driver,
            gripper_node,
            realsense_camera_top,
            realsense_camera_left,
            realsense_camera_right,
            # depth_image_proc 点云节点 (替代 realsense 自带点云，在 color 坐标系下生成)
            _make_pc_node("camera_top"),
            _make_pc_node("camera_left"),
            _make_pc_node("camera_right"),
            enable_arms,
        ])

    return LaunchDescription(
        ld
    )

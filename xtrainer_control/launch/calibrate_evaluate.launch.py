from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_name = "top_435" # camera_name

    aruco_single_params = {
        "image_is_rectified": True,
        "marker_id": 99,
        "marker_size": 0.078,
        "reference_frame": f"camera_{arg_name}_link",
        # 彩色图像的 PnP 结果位于彩色光学坐标系；aruco_ros 再通过
        # RealSense 发布的固定 TF 将 marker pose 转到 reference_frame。
        "camera_frame": f"camera_{arg_name}_color_optical_frame",
        "marker_frame": f"aruco_marker_{arg_name}",
        "corner_refinement": "LINES",
    }
    aruco_camera_info_topic_name = f"/camera/camera_{arg_name}/color/camera_info"
    aruco_image_topic_name = f"/camera/camera_{arg_name}/color/image_raw"


    handeye_rqt_evaluator = Node(package='easy_handeye2', executable='rqt_evaluator.py',
                                  name='handeye_rqt_evaluator',
                                  # arguments=['--ros-args', '--log-level', 'debug'],
                                  parameters=[{
                                      'name': f"{arg_name}_cam_cal",
                                  }])

    return LaunchDescription([
        handeye_rqt_evaluator,
        Node(
            package="aruco_ros",
            executable="single",
            parameters=[aruco_single_params],
            remappings=[
                ("/camera_info", aruco_camera_info_topic_name),
                ("/image", aruco_image_topic_name),
            ],
        ),
    ])

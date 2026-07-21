# Dobot_ROS2_SDK

## Commands

### dobot_bringup_serive

```bash
# PowerOn
ros2 service call /Arm1/dobot_bringup_ros2/srv/PowerOn dobot_msgs_v4/srv/PowerOn "{}"
ros2 service call /Arm2/dobot_bringup_ros2/srv/PowerOn dobot_msgs_v4/srv/PowerOn "{}"
# Enable
ros2 service call /Arm1/dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot "{}"
ros2 service call /Arm2/dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot "{}"
# PowerOff
ros2 service call /Arm1/dobot_bringup_ros2/srv/DisableRobot dobot_msgs_v4/srv/DisableRobot "{}"
ros2 service call /Arm2/dobot_bringup_ros2/srv/DisableRobot dobot_msgs_v4/srv/DisableRobot "{}"
```

## Ideal grasp pose

```bash
[read_pose-1] [INFO] [1784624027.614053457] [read_pose_node]: Arm1 — pos:
[read_pose-1] x: 0.5041
[read_pose-1] y: 0.0192
[read_pose-1] z: 0.1379
[read_pose-1] ox: 0.5010
[read_pose-1] oy: -0.4980
[read_pose-1] oz: 0.5240
[read_pose-1] ow: -0.47587153899160844)
[read_pose-1] [INFO] [1784624027.614462020] [read_pose_node]: Arm1 — pos:
[read_pose-1] x: 0.5204
[read_pose-1] y: 0.0053
[read_pose-1] z: 0.2588
[read_pose-1] ox: 0.6927
[read_pose-1] oy: 0.7211
[read_pose-1] oz: 0.0060
[read_pose-1] ow: -0.014223606929005907)
```

## Important Topic

### Camera

* /camera/camera_left/color/image_raw
* /camera/camera_left/depth/image_rect_raw
* /camera/camera_right/color/image_raw
* /camera/camera_right/depth/image_rect_raw
* /camera/camera_top/color/image_raw
* /camera/camera_top/depth/image_rect_raw

## TF Links

### Robot

* base_link
* L1_6, L2_6
* L1_gripper_tcp, L1_gripper_tcp

### Camera

* Camera base: camera_left_link, camera_right_link
* Camera color: camera_left_color_optical_frame, camera_right_color_optical_frame
* Camera color: camera_left_depth_optical_frame, camera_right_depth_optical_frame

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

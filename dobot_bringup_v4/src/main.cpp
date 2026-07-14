

#include <dobot_bringup/cr_robot_ros2.h>
#include <dobot_msgs_v4/msg/robot_status.hpp>
#include <dobot_msgs_v4/msg/tool_vector_actual.hpp>
#include <iostream>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<CRRobotRos2>();

    // 从参数中读取 joint_names，默认为 joint1~joint6
    std::vector<std::string> joint_names = robot->declare_parameter<std::vector<std::string>>("joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    int joint_count = joint_names.size();

    // 创建关节状态消息和发布者
    sensor_msgs::msg::JointState joint_state_msg;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub = robot->create_publisher<sensor_msgs::msg::JointState>("joint_states_robot", 10);
    joint_state_msg.name = joint_names;
    joint_state_msg.position = std::vector<double>(joint_count, 0.0);

    dobot_msgs_v4::msg::RobotStatus robot_status_msg;
    rclcpp::Publisher<dobot_msgs_v4::msg::RobotStatus>::SharedPtr robot_status_pub = robot->create_publisher<dobot_msgs_v4::msg::RobotStatus>("dobot_msgs_v4/msg/RobotStatus", 10);

    dobot_msgs_v4::msg::ToolVectorActual tool_vector_actual_msg;
    rclcpp::Publisher<dobot_msgs_v4::msg::ToolVectorActual>::SharedPtr tool_vector_pub = robot->create_publisher<dobot_msgs_v4::msg::ToolVectorActual>("dobot_msgs_v4/msg/ToolVectorActual", 10);

    std::string z = "/";
    const char *robot_type = getenv("DOBOT_TYPE");
    std::string a = robot_type == nullptr ? "cr5" : robot_type;
    std::string b = "_robot/joint_controller/follow_joint_trajectory";
    std::string ss = z + a + b;
    //  for (uint32_t i = 0; i < 6; i++)
    //  {
    //    joint_state_msg.position.push_back(0.0);
    //    joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
    //  }

    double rate_value = robot->declare_parameter<double>("JointStatePublishRate", 10.0);

    robot->init();

    rclcpp::Rate rate(rate_value);
    double position[6];
    while (rclcpp::ok())
    {
        // 获取关节状态并发布消息
        if (robot->isConnected())
        {
            robot->getJointState(position);
        }
        else
        {
            memset(position, 0, sizeof(position));
        }
        joint_state_msg.header.stamp = robot->get_clock()->now();
        joint_state_msg.header.frame_id = "dummy_link";
        for (uint32_t i = 0; i < 6; i++)
        {
            joint_state_msg.position[i] = position[i];
        }
        joint_state_pub->publish(joint_state_msg);

        double val[6];
        if (robot->isConnected())
        {
            robot->getToolVectorActual(val);
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_pub->publish(tool_vector_actual_msg);
        }

        // publish robot status
        robot_status_msg.is_enable = robot->isEnable();
        robot_status_msg.is_connected = robot->isConnected();
        robot_status_pub->publish(robot_status_msg);
        rclcpp::spin_some(robot);
        rate.sleep();
    }

    // 关闭 ROS 2 环境
    rclcpp::shutdown();
    return 0;
}

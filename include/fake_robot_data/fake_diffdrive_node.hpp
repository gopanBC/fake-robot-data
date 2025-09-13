#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"


class FakeDiffDriveNode : public rclcpp::Node
{
public:
FakeDiffDriveNode();


private:
void timerCallback();
void publishOdometry();
void publishImu();


rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
rclcpp::TimerBase::SharedPtr timer_;


std::string motion_type_;
double linear_speed_, angular_speed_;
double target_distance_, target_angle_;


double x_, y_, theta_;
rclcpp::Time start_time_;
};
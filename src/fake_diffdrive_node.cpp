#include "fake_diffdrive_node.hpp"


void FakeDiffDriveNode::timerCallback()
{
double dt = 0.05;
geometry_msgs::msg::Twist cmd;


if (motion_type_ == "straight") {
if (x_ < target_distance_) {
cmd.linear.x = linear_speed_;
x_ += linear_speed_ * dt;
}
} else if (motion_type_ == "turn") {
if (std::fabs(theta_) < std::fabs(target_angle_)) {
cmd.angular.z = (target_angle_ > 0 ? angular_speed_ : -angular_speed_);
theta_ += cmd.angular.z * dt;
}
} else if (motion_type_ == "curve") {
cmd.linear.x = linear_speed_;
cmd.angular.z = angular_speed_;
x_ += linear_speed_ * dt * std::cos(theta_);
y_ += linear_speed_ * dt * std::sin(theta_);
theta_ += angular_speed_ * dt;
}


cmd_pub_->publish(cmd);
publishOdometry();
publishImu();
}


void FakeDiffDriveNode::publishOdometry()
{
auto msg = nav_msgs::msg::Odometry();
msg.header.stamp = now();
msg.header.frame_id = "odom";
msg.child_frame_id = "base_link";
msg.pose.pose.position.x = x_;
msg.pose.pose.position.y = y_;


tf2::Quaternion q;
q.setRPY(0, 0, theta_);
msg.pose.pose.orientation.x = q.x();
msg.pose.pose.orientation.y = q.y();
msg.pose.pose.orientation.z = q.z();
msg.pose.pose.orientation.w = q.w();


odom_pub_->publish(msg);
}


void FakeDiffDriveNode::publishImu()
{
auto msg = sensor_msgs::msg::Imu();
msg.header.stamp = now();
msg.header.frame_id = "base_link";


tf2::Quaternion q;
q.setRPY(0, 0, theta_);
msg.orientation.x = q.x();
msg.orientation.y = q.y();
msg.orientation.z = q.z();
msg.orientation.w = q.w();


msg.angular_velocity.z = angular_speed_;
imu_pub_->publish(msg);
}


int main(int argc, char *argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<FakeDiffDriveNode>());
rclcpp::shutdown();
return 0;
}
// #pragma once
// #include <memory>
// #include <string>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"

// class PursuitController : public rclcpp::Node {
//     public:
//     PursuitController(int robot_i, int target_id);

//     private:
//     void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     void targetOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q);
//     void controlLoop();

//     int robot_id_;
//     int target_id_;
//     double pursuit_distance_;
//     double max_linear_speed_;
//     double max_angular_speed_;

//     nav_msgs::msg::Odometry::SharedPtr my_odom_;
//     nav_msgs::msg::Odometry::SharedPtr target_odom_;

//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_odom_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;


// };
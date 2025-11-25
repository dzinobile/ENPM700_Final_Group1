#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class FrameListener : public rclcpp::Node {

    public:
    FrameListener();

    private:
    void onTimer();
    void controlRobot(const std::string &child_frame,
                      const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &pub,
                      const std::string &target_frame,
                      bool is_leader);

    int num_robots_;
    std::vector<std::string> target_frames_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};
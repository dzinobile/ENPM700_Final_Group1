#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node {

    public: 
    SimplePublisher();

    private:
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    int num_bots_;

};

#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "States.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class ExploreState;
class EncircleState;
class HerdState;

class SheepdogNode : public rclcpp::Node {
    public:
    SheepdogNode();

    void run();
    void changeState(States* newState);
    bool sheepDetected() const {return sheep_detected_;}
    double getDistanceToSheep() const {return distance_to_sheep_;}


    States* getCurrentState() const {
        return currentState_.get();
    }
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    private:
    std::unique_ptr<States> currentState_;
    rclcpp::TimerBase::SharedPtr timer_;
    double sheep_detection_radius_;
    double distance_to_sheep_;


    bool sheep_detected_;
    void updateSheepDetection();
    void updateCallback();


};
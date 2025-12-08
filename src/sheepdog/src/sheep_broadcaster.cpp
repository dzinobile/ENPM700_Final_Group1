/**
 * @file sheep_broadcaster.cpp
 * @brief Publishes a TF transform from the map frame to a fixed sheep frame at
 * a fixed rate.
 * @author Anvesh Som
 *
 * @copyright
 * Copyright (c) 2025 Sheepdog Project Team.
 * Licensed under the Apache License, Version 2.0. See the LICENSE file in the
 * project root for details.
 */

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @class MapToSheepBroadcaster
 * @brief ROS 2 node that periodically broadcasts a TF transform from "map" to
 * "sheep".
 *
 * The node publishes a geometry_msgs::msg::TransformStamped message at a fixed
 * rate to represent the pose of a sheep in the map frame. This can be used by
 * other nodes to reason about the relative position of the sheep.
 */
class MapToSheepBroadcaster : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new MapToSheepBroadcaster node.
   *
   * Creates a TF2 transform broadcaster and a timer that triggers periodic
   * publication of the map-to-sheep transform at 10 Hz.
   */
  MapToSheepBroadcaster() : Node("sheep_broadcaster") {
    // Read use_sim_time if already declared externally
    bool use_sim_time = this->get_parameter_or("use_sim_time", false);

    if (!use_sim_time) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }

    
    // Create a TransformBroadcaster object
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer to publish at 10 Hz
    timer_ = this->create_wall_timer(
        100ms, std::bind(&MapToSheepBroadcaster::on_timer, this));
  }

 private:
  /**
   * @brief Timer callback that publishes the map-to-sheep transform.
   *
   * The transform is published with the parent frame "map" and child frame
   * "sheep". The translation and rotation represent the current pose of the
   * sheep in the map frame. In this implementation, the pose is fixed and
   * hard-coded.
   */
  void on_timer() {
    geometry_msgs::msg::TransformStamped sheep_tf_msg;
    geometry_msgs::msg::TransformStamped pen_tf_msg;

    // Timestamp and frame ids
    sheep_tf_msg.header.stamp = this->get_clock()->now();
    sheep_tf_msg.header.frame_id = "map";   ///< Parent frame.
    sheep_tf_msg.child_frame_id = "sheep";  ///< Child frame.

    // Example translation (change as needed)
    sheep_tf_msg.transform.translation.x = -4.0;
    sheep_tf_msg.transform.translation.y = 1.0;
    sheep_tf_msg.transform.translation.z = 0.0;

    // Example rotation: roll = 0, pitch = 0, yaw = 0.5 rad
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.5);
    q.normalize();

    sheep_tf_msg.transform.rotation.x = q.x();
    sheep_tf_msg.transform.rotation.y = q.y();
    sheep_tf_msg.transform.rotation.z = q.z();
    sheep_tf_msg.transform.rotation.w = q.w();


    pen_tf_msg.header.stamp = this->get_clock()->now();
    pen_tf_msg.header.frame_id = "map";
    pen_tf_msg.child_frame_id = "pen";

    pen_tf_msg.transform.translation.x = 0.0;
    pen_tf_msg.transform.translation.y = 0.0;
    pen_tf_msg.transform.translation.z = 0.0;

    pen_tf_msg.transform.rotation.x = q.x();
    pen_tf_msg.transform.rotation.y = q.y();
    pen_tf_msg.transform.rotation.z = q.z();
    pen_tf_msg.transform.rotation.w = q.w();

    // Publish the transform
    tf_broadcaster_->sendTransform(sheep_tf_msg);
    tf_broadcaster_->sendTransform(pen_tf_msg);
  }

  /**
   * @brief TF2 transform broadcaster used to send the map-to-sheep transform.
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /**
   * @brief Timer that triggers periodic publishing of the transform.
   */
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Entry point for the MapToSheepBroadcaster node.
 *
 * Initializes the ROS 2 system, instantiates the MapToSheepBroadcaster node,
 * and starts spinning until shutdown is requested.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Exit code (0 on normal shutdown).
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapToSheepBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

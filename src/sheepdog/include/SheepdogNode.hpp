// Copyright 2025
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @file SheepdogNode.hpp
 * @brief Header file for State Interface class
 * @author Daniel Zinobile
 * @date 08-Dec-2025
 */
#pragma once

#include "States.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>

// Forward declarations to states avoid circular references
class ExploreState;
class EncircleState;
class HerdState;

/**
 * @class SheepdogNode
 * @brief State Interface class through which all machine states are run. the
 * node:
 *  - Runs per current state
 *  - Changes state based on current conditions
 *  - Checks if the sheep has been detected
 *  - Continuously gets the distance from the robot to the sheep
 *  - Gets current state
 */
class SheepdogNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the Sheepdog node
   */
  SheepdogNode();

  /**
   * @brief Function to start the node in a blocking loop
   */
  void run();

  /**
   * @brief Function to change the state to the given state
   * @param newState new state to change to
   */
  void changeState(States *newState);

  /**
   * @brief getter for sheep detected boolean value
   * @return Boolean for whether sheep has been detected
   */
  bool sheepDetected() const { return sheep_detected_; }

  /**
   * @brief Getter for current distance to sheep
   * @return Current distance to sheep
   */
  double getDistanceToSheep() const { return distance_to_sheep_; }

  /**
   * @brief Getter for current machine state
   * @return current machine state
   */
  States *getCurrentState() const { return currentState_.get(); }

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
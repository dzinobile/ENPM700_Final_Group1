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
// See the License for the specific
/**
 * @file ExploreState.hpp
 * @brief Node for Explore state.
 * The Sheepdog Node initializes in this state.
 * In this state, the robot travels in a spiraling search pattern,
 * Using SLAM to build a map as it goes.
 * Once the sheep is in a certain range, the sheepdog node switches to the
 * Encircle state.
 */
#pragma once

#include "States.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>

/**
 * @class ExploreState
 * @brief State in which the robot explores the environment in a sprial pattern
 * while searching for the sheep
 */
class ExploreState : public States {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor for Explore state
   */
  ExploreState();

  /**
   * @brief Inherited Desctructor for Explore state
   */
  ~ExploreState() override;

  /**
   * @brief Inherited update function to activate behaviors for current state
   * @param context Reference to SheepdogNode
   */
  void update(SheepdogNode &context) override;

  /**
   * @brief Inherited transition function to change state based on current
   * conditions
   * @param context Reference to SheepdogNode
   */
  States *transition(SheepdogNode &context) override;

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  std::vector<std::vector<int>> directions_;
  double step_length_;
  double step_increment_;
  double x_;
  double y_;
  int count_;

  double map_width_;
  double map_height_;
  double map_min_x_;
  double map_max_x_;
  double map_min_y_;
  double map_max_y_;

  bool goal_active_;
  bool goal_reached_;

  std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  // Helper methods
  /**
   * @brief Function to initialize a navigation client
   * @param context Reference to SheepdogNode
   */
  void initializeNavClient(SheepdogNode &context);

  /**
   * @brief Function to send next goal in spiral pattern once current one is
   * reached
   * @param context Reference to SheepdogNode
   */
  void sendNextGoal(SheepdogNode &context);

  /**
   * @brief Function to check if goal point is in bounds of the costmap
   * @param x goal x value
   * @param y goal y value
   * @return boolean for if point is in bounds
   */
  bool isPointInBounds(double x, double y) const;

  /**
   * @brief Function to clamp goal to costmap bounds
   * @param x reference to goal x value
   * @param y reference to goal y value
   */
  void clampGoalToBounds(double &x, double &y);

  /**
   * @brief Callback function invoked when the action server responds to a goal
   * request
   * @param goal_handle Shared pointer to the goal handle returned by the action
   * server
   */
  void goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle);

  /**
   * @brief Callback function invoked when the action server finishes processing
   * a goal
   * @param result the result wrapper returned by the action server containing
   * the status and any output data from the goal execution.
   */
  void goalResultCallback(const GoalHandleNav::WrappedResult &result);
};
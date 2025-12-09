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
 * @file EncircleState.hpp
 * @brief Node for Encircle state.
 * This State activates when the sheep is detected by the robot.
 * In this state, the robot positions itself a certain offset distance away from
 * the sheep.
 * @author Daniel Zinobile
 * @date 08-Dec-2025
 */
#pragma once

#include "States.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>

// Forward declaration to avoid circular references
class SheepdogNode;

/**
 * @class EncircleState
 * @brief State in which the robot positions itself to encircle the sheep
 */
class EncircleState : public States {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  /**
   * @brief Constructor for Encircle state
   */
  EncircleState();

  /**
   * @brief Inherited destructor for Encircle state
   */
  ~EncircleState() override;

  /**
   * @brief Inherited update function to activate behaviors for current state
   * @param context Reference to SheepdogNode
   */
  void update(SheepdogNode &context) override;

  /**
   * @brief Inherited transition function to change state based on current
   * conditions
   * @param context Reference to SheepdogNode
   * @return Reference to state
   */
  States *transition(SheepdogNode &context) override;

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  bool goal_sent_;
  bool goal_reached_;
  double offset_distance_;

  /**
   * @brief Function to initialize a navigation client
   * @param context Reference to SheepdogNode
   */
  void initializeNavClient(SheepdogNode &context);

  /**
   * @brief Function to send goal for encircling sheep
   * @param context Reference to SheepdogNode
   */
  void sendEncircleGoal(SheepdogNode &context);

  /**
   * @brief Callback function invoked when the action server responds to a goal
   * request.
   *
   * This is called after sending a goal to the server to check whether the
   * server has accepted or rejected the goal.
   *
   * @param goal_handle Shared pointer to the goal handle returned by the action
   * server.
   */
  void goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle);
  /**
   * @brief Callback function invoked when the action server finishes processing
   * a goal.
   *
   * This is called when the server has completed the goal (succeeded, aborted,
   * or canceled) and provides the result of the action.
   *
   * @param result The result wrapper returned by the action server containing
   * the status and any output data from the goal execution.
   */
  void goalResultCallback(const GoalHandleNav::WrappedResult &result);
};
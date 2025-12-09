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
 * @file HerdState.hpp
 * @brief Node for Herd state.
 * This state activates when the robot has positioned itself to encircle the
 * sheep. In this state, the robot moves back to the location of the pen, offset
 * by the same amount As the Encircle offset to ensure the sheep is encircled
 * while all robots are moving.
 * @author Daniel Zinobile
 * @date 08-Dec-2025
 */
#pragma once

#include "States.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// Forward declaration to avoid circular references
class SheepdogNode;

/**
 * @class HerdState
 * @brief State in which the robots travel back to the pen while encircling the
 * sheep. This state activates once the robot has reached its Encircle position.
 */
class HerdState : public States {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  /**
   * @brief Constructor for Herd state
   */
  HerdState();

  /**
   * @brief Inherited destructor for Herd state
   */
  ~HerdState() override;

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
   * @brief Function to send goal for moving back to pen
   * @param context Reference to SheepdogNode
   */
  void sendHerdGoal(SheepdogNode &context);

  /**
   * @brief Callback function invoked when the action server responds to a goal
   * request.
   * @param goal_handle Shared pointer to the goal handle returned by the action
   * server.
   */
  void goalResponseCallback(const GoalHandleNav::SharedPtr &goal_handle);

  /**
   * @brief Callback function invoked when the action server finishes processing
   * a goal.
   * @param result The result wrapper returned by the action server containing
   * the status and any output data from the goal execution.
   */
  void goalResultCallback(const GoalHandleNav::WrappedResult &result);
};
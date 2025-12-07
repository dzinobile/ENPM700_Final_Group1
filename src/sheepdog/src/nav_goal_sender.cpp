/**
 * @file nav_goal_sender.cpp
 * @brief Sends a sequence of Nav2 goals that drive the robot in an expanding
 * spiral pattern in the map frame.
 * @author Daniel Zinobile
 *
 * @copyright
 * Copyright (c) 2025 Sheepdog Project Team.
 * Licensed under the Apache License, Version 2.0. See the LICENSE file in the
 * project root for details.
 */

#include <chrono>
#include <memory>
#include <string>
#include <tuple>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

/**
 * @class NavGoalSender
 * @brief ROS 2 node that issues NavigateToPose goals to the Nav2 stack in an
 * expanding spiral pattern.
 *
 * The node waits for the Nav2 action server to become available and then
 * repeatedly sends goals in four cardinal directions while gradually increasing
 * step length. This generates a simple outward spiral exploration behavior in
 * the map frame.
 */
class NavGoalSender : public rclcpp::Node {
 public:
  /// Alias for the NavigateToPose action type used by Nav2.
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  /// Alias for the goal handle associated with NavigateToPose.
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Construct a new NavGoalSender node.
   *
   * Initializes the NavigateToPose action client, sets up the spiral pattern
   * parameters, and starts waiting for the Nav2 action server before sending
   * the first goal.
   */
  NavGoalSender() : Node("nav_goal_sender") {
    // Declare parameters for map bounds
    this->declare_parameter("map_width", 20.0);   // meters
    this->declare_parameter("map_height", 20.0);  // meters
    this->declare_parameter("step_length", 1);
    this->declare_parameter("step_increment", 1);
    
    // Get parameters
    map_width_ = this->get_parameter("map_width").as_double();
    map_height_ = this->get_parameter("map_height").as_double();
    step_length_ = this->get_parameter("step_length").as_int();
    step_increment_ = this->get_parameter("step_increment").as_int();
    
    // Map is centered at origin, so bounds are +/- half width/height
    map_min_x_ = -map_width_ / 2.0;
    map_max_x_ = map_width_ / 2.0;
    map_min_y_ = -map_height_ / 2.0;
    map_max_y_ = map_height_ / 2.0;

    client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    directions_ = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    
    x_ = 0;
    y_ = 0;
    count_ = 0;

    RCLCPP_INFO(this->get_logger(), 
                "NavGoalSender initialized with map bounds:");
    RCLCPP_INFO(this->get_logger(), 
                "  X: [%.1f, %.1f] meters", map_min_x_, map_max_x_);
    RCLCPP_INFO(this->get_logger(), 
                "  Y: [%.1f, %.1f] meters", map_min_y_, map_max_y_);
    RCLCPP_INFO(this->get_logger(), 
                "  step_length: %d, step_increment: %d", 
                step_length_, step_increment_);

    // Kick off once Nav2 is connected
    wait_for_nav2();
  }

 private:
  /**
   * @brief NavigateToPose action client used to send goals to Nav2.
   */
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  /**
   * @brief Discrete movement directions for the spiral pattern.
   *
   * The four entries represent unit steps in the order:
   * (1, 0) -> (0, 1) -> (-1, 0) -> (0, -1), i.e. +x, +y, -x, -y.
   */
  std::vector<std::vector<int>> directions_;

  /**
   * @brief Current step length used for each pair of directions in the spiral.
   *
   * This value grows over time to generate an outward spiral path.
   */
  int step_length_;

  /**
   * @brief Amount by which to increment step_length_ after each pair of moves.
   *
   * Higher values create wider spacing between spiral loops.
   */
  int step_increment_;

  /**
   * @brief Current x position of the goal in the map frame.
   *
   * This is expressed in map coordinates and is updated incrementally.
   */
  int x_;

  /**
   * @brief Current y position of the goal in the map frame.
   *
   * This is expressed in map coordinates and is updated incrementally.
   */
  int y_;

  /**
   * @brief Counter used to index directions and update step length.
   *
   * The value is incremented every time a goal result is received, and is
   * used to select the current direction and determine when to grow the step
   * length.
   */
  int count_;

  /**
   * @brief Known map boundaries (in meters).
   */
  double map_width_;
  double map_height_;
  double map_min_x_;
  double map_max_x_;
  double map_min_y_;
  double map_max_y_;

  /**
   * @brief Check if a point is within the known map bounds.
   * Assumes everything within bounds is free space (optimistic).
   */
  bool isPointInBounds(int x, int y) {
    return (x >= map_min_x_ && x <= map_max_x_ &&
            y >= map_min_y_ && y <= map_max_y_);
  }

  /**
   * @brief Clamp a goal to be within map bounds.
   * @param x Goal x coordinate (modified in place)
   * @param y Goal y coordinate (modified in place)
   */
  void clampGoalToBounds(int& x, int& y) {
    int original_x = x;
    int original_y = y;
    
    if (x < map_min_x_) x = static_cast<int>(map_min_x_);
    if (x > map_max_x_) x = static_cast<int>(map_max_x_);
    if (y < map_min_y_) y = static_cast<int>(map_min_y_);
    if (y > map_max_y_) y = static_cast<int>(map_max_y_);
    
    if (x != original_x || y != original_y) {
      RCLCPP_INFO(this->get_logger(), 
                  "Clamped goal from (%d, %d) to (%d, %d) to stay within bounds",
                  original_x, original_y, x, y);
    }
  }

  /**
   * @brief Wait for the Nav2 NavigateToPose action server and start sending
   * goals.
   *
   * This function blocks for up to 10 seconds for the action server. If the
   * server is not available within that time, an error is logged and the ROS
   * graph is shut down. If the server becomes available, the first goal in the
   * spiral sequence is sent.
   */
  void wait_for_nav2() {
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Nav2 connected. Starting spiral pattern...");
    send_new_goal();
  }

  /**
   * @brief Compute and send the next goal in the spiral pattern.
   *
   * The method updates the internal spiral state (direction, step length, and
   * current (x, y) position), constructs a NavigateToPose goal in the map
   * frame, and sends it asynchronously via the Nav2 action client. The result
   * callback schedules the next goal regardless of success or failure, which
   * maintains continuous exploration.
   */
  void send_new_goal() {
    int mod2 = count_ % 2;
    int mod4 = count_ % 4;

    (void)mod2;  // currently unused but kept for potential future logic

    x_ += step_length_ * directions_[mod4][0];
    y_ += step_length_ * directions_[mod4][1];

    // Clamp goal to known map bounds
    int clamped_x = x_;
    int clamped_y = y_;
    clampGoalToBounds(clamped_x, clamped_y);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose.position.x = clamped_x;
    goal_msg.pose.pose.position.y = clamped_y;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal: (%d, %d), step_length: %d", 
                clamped_x, clamped_y, step_length_);

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    /**
     * @brief Result callback for the NavigateToPose action.
     *
     * On success or failure, the internal spiral state is updated and the next
     * goal in the sequence is sent. This guarantees continued navigation even
     * if individual goals fail.
     *
     * @param result Wrapped result of the NavigateToPose action.
     */
    send_goal_options.result_callback =
        [this](const GoalHandleNav::WrappedResult& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Sending next...");
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to reach goal (code %d). Sending next...",
                        static_cast<int>(result.code));
          }
          count_++;
          step_length_ += (count_ % 2) * step_increment_;
          send_new_goal();
        };

    client_->async_send_goal(goal_msg, send_goal_options);
  }
};

/**
 * @brief Entry point for the NavGoalSender node.
 *
 * Initializes the ROS 2 system, instantiates the NavGoalSender node,
 * and starts spinning until shutdown is requested.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Exit code (0 on normal shutdown).
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavGoalSender>());
  rclcpp::shutdown();
  return 0;
}
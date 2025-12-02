// Copyright ...
#include <catch_ros2/catch_ros2.hpp>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

auto Logger = rclcpp::get_logger("");

class GoalSendTestFixture {
public:
  GoalSendTestFixture() {
    testerNode = rclcpp::Node::make_shared("goal_send_test_node");
    Logger = testerNode->get_logger();

    testerNode->declare_parameter("test_duration", 20.0);
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value().get<double>();

    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        testerNode, "navigate_to_pose");

    RCLCPP_INFO(Logger, "Test duration: %.2f seconds", TEST_DURATION);
  }

protected:
  rclcpp::Node::SharedPtr testerNode;
  double TEST_DURATION;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
};

TEST_CASE_METHOD(GoalSendTestFixture,
                 "Test basic goal sending to Nav2",
                 "[navigation]") {

  // Wait for Nav2 action server
  RCLCPP_INFO(Logger, "Waiting for Nav2 action server...");
  bool found = client_->wait_for_action_server(std::chrono::duration<double>(TEST_DURATION));
  REQUIRE(found);
  RCLCPP_INFO(Logger, "Nav2 action server AVAILABLE!");

  // Create a simple goal (move a tiny distance)
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = testerNode->now();
  goal.pose.pose.position.x = 0.5;
  goal.pose.pose.position.y = 0.0;
  goal.pose.pose.orientation.w = 1.0;

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  std::shared_future<rclcpp_action::ClientGoalHandle<
      nav2_msgs::action::NavigateToPose>::WrappedResult> result_future;

  send_goal_options.result_callback =
      [&](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
        result_future = std::async(std::launch::deferred, [result]() { return result; });
      };

  auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
          testerNode, goal_handle_future, std::chrono::duration<double>(TEST_DURATION)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    FAIL("Sending goal timed out!");
  }

  auto goal_handle = goal_handle_future.get();
  REQUIRE(goal_handle != nullptr);

  // Wait for result
  if (rclcpp::spin_until_future_complete(
          testerNode, result_future, std::chrono::duration<double>(TEST_DURATION)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    FAIL("Goal result timed out!");
  }

  auto result = result_future.get();
  RCLCPP_INFO(Logger, "Goal result received with code: %d", result.code);

  REQUIRE(result.code == rclcpp_action::ResultCode::SUCCEEDED);
}

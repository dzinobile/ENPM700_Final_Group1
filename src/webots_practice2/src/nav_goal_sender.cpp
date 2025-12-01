#include <chrono>
#include <memory>
#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class NavGoalSender : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavGoalSender()
  : Node("nav_goal_sender")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    directions_ = {{1,0},{0,1},{-1,0},{0,-1}};
    step_length_ = 1;
    x_ = 0;
    y_ = 0;
    count_ = 0;

    // Kick off once Nav2 is connected
    wait_for_nav2();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::vector<std::vector<int>> directions_;
  int step_length_;
  int x_;
  int y_;
  int count_;

  void wait_for_nav2()
  {
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
      rclcpp::shutdown();
      return;
    }
    send_new_goal();
  }

  void send_new_goal()
  {
    int mod2 = count_ % 2;
    int mod4 = count_ % 4;

    x_ += step_length_ * directions_[mod4][0];
    y_ += step_length_ * directions_[mod4][1];

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose.position.x = x_;
    goal_msg.pose.pose.position.y = y_;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal: (%d, %d)", x_, y_);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    // This callback triggers when navigation is complete
    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Goal reached! Sending next...");
          count_++;
          step_length_ += count_ % 2;
          send_new_goal();
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to reach goal. Retrying next...");
          count_++;
          step_length_ += count_ % 2;
          send_new_goal();
        }
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavGoalSender>());
  rclcpp::shutdown();
  return 0;
}

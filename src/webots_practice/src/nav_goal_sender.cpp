#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

    timer_ = this->create_wall_timer(
      2s, std::bind(&NavGoalSender::send_goal, this));
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_goal()
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();

    // Set your target location here!
    goal_msg.pose.pose.position.x = 1.0;
    goal_msg.pose.pose.position.y = 0.0;

    // No rotation (facing forward)
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal!");
    client_->async_send_goal(goal_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavGoalSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "EncircleState.hpp"
#include "SheepdogNode.hpp"
#include "HerdState.hpp"
#include "ExploreState.hpp"

EncircleState::EncircleState() :
    goal_sent_(false),
    goal_reached_(false),
    offset_distance_(0.5) {}

EncircleState::~EncircleState(){}

void EncircleState::update(SheepdogNode &context) {
    initializeNavClient(context);

    if(!goal_sent_) {
        sendEncircleGoal(context);
    }

}

States* EncircleState::transition(SheepdogNode &context) {
    if (!context.sheepDetected()) {
        return new ExploreState();
    }

    if (goal_reached_) {
        return new HerdState();
    }

    return nullptr;

}

void EncircleState::initializeNavClient(SheepdogNode &context) {
    if (!nav_client_) {
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            context.shared_from_this(),
            "navigate_to_pose"
        );
    }
}

void EncircleState::sendEncircleGoal(SheepdogNode &context) {
    geometry_msgs::msg::TransformStamped sheep_tf;

    try {
        sheep_tf = context.tf_buffer_->lookupTransform(
            "map",
            "sheep",
            tf2::TimePointZero
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(context.get_logger(), "could not get sheep transform");
        return;
    }

    double target_x = sheep_tf.transform.translation.x;
    double target_y = sheep_tf.transform.translation.y - offset_distance_;

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = context.get_clock()->now();

    goal_pose.pose.position.x = target_x;
    goal_pose.pose.position.y = target_y;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.w = 1.0;

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&EncircleState::goalResponseCallback, this, std::placeholders::_1 );
    send_goal_options.result_callback = 
        std::bind(&EncircleState::goalResultCallback, this, std::placeholders::_1);
    
    nav_client_->async_send_goal(goal, send_goal_options);

    goal_sent_ = true;
    goal_reached_ = false;

    RCLCPP_INFO(context.get_logger(), "Encircle goal sent");



}

void EncircleState::goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle) {
    if (!goal_handle) {
        goal_sent_ = false;
    } else {
        current_goal_handle_ = goal_handle;
    }
}

void EncircleState::goalResultCallback(
    const GoalHandleNav::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            goal_reached_ = true;
        } else {
            goal_sent_ = false;
            goal_reached_ = false;
        }
    }


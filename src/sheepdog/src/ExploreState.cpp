#include "ExploreState.hpp"
#include "SheepdogNode.hpp"
#include "EncircleState.hpp"

using namespace std::chrono_literals;

ExploreState::ExploreState() :
    step_length_(0.5),
    step_increment_(1),
    x_(0),
    y_(0),
    count_(0),
    map_width_(20.0),
    map_height_(20.0),
    goal_active_(false),
    goal_reached_(false) {
        directions_ = {{1,0}, {0,1}, {-1,0}, {0,-1}};

        map_min_x_ = -map_width_ / 2.0;
        map_max_x_ = map_width_ / 2.0;
        map_min_y_ = -map_height_ / 2.0;
        map_max_y_ = map_height_ / 2.0;
    }

    ExploreState::~ExploreState() {
        if (goal_active_ && current_goal_handle_) {
            nav_client_->async_cancel_goal(current_goal_handle_);
        }
    }

    void ExploreState::update(SheepdogNode &context) {
        if (!nav_client_) {
            initializeNavClient(context);
            return;
        }

        if (!goal_active_) {
            sendNextGoal(context);
        }

    }
States* ExploreState::transition(SheepdogNode &context) {
    if (context.sheepDetected()) {
        RCLCPP_INFO(context.get_logger(), 
        "Sheep detected, transition to ENCIRCLE state");
        return new EncircleState();
    }

    return nullptr;
}

void ExploreState::initializeNavClient(SheepdogNode &context) {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        &context, "navigate_to_pose"
    );

    RCLCPP_INFO(context.get_logger(), 
                "EXPLORE: Waiting for nax2 action server...");
    if (nav_client_->wait_for_action_server(10s)) {
        RCLCPP_INFO(context.get_logger(), 
                    "EXPLORE: nav2 connected. Starting spiral exploration...");
    } else {
        RCLCPP_ERROR(context.get_logger(), 
                    "EXPLORE: Nav2 action server not available");
    }
}

void ExploreState::sendNextGoal(SheepdogNode &context) {
// Update spiral position
    int mod4 = count_ % 4;
    x_ += step_length_ * directions_[mod4][0];
    y_ += step_length_ * directions_[mod4][1];
    
    // Clamp to map bounds
    double clamped_x = x_;
    double clamped_y = y_;
    clampGoalToBounds(clamped_x, clamped_y);
    
    // Create goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = context.now();
    goal_msg.pose.pose.position.x = static_cast<double>(clamped_x);
    goal_msg.pose.pose.position.y = static_cast<double>(clamped_y);
    goal_msg.pose.pose.orientation.w = 1.0;
    
    RCLCPP_INFO(context.get_logger(), 
                "EXPLORE: Sending goal (%f, %f), step_length: %f",
                clamped_x, clamped_y, step_length_);
    
    // Setup callbacks
    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = 
        std::bind(&ExploreState::goalResponseCallback, this, 
                  std::placeholders::_1);
    
    send_goal_options.result_callback = 
        std::bind(&ExploreState::goalResultCallback, this, 
                  std::placeholders::_1);
    
    // Send goal
    goal_active_ = true;
    goal_reached_ = false;
    goal_handle_future_ = nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void ExploreState::goalResponseCallback(
    const GoalHandleNav::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(rclcpp::get_logger("ExploreState"), 
                                            "goal was rejected by server");
            goal_active_ = false;
        } else {
            current_goal_handle_ = goal_handle;
            RCLCPP_INFO(rclcpp::get_logger("ExploreState"),
                                            "goal accepted by server");
        }
    }
void ExploreState::goalResultCallback(
    const GoalHandleNav::WrappedResult& result) {
        goal_active_ = false;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(rclcpp::get_logger("ExploreState"),
                                            "goal reached successfully");
            goal_reached_ = true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ExploreState"),
                                            "failed to reach goal (code %d)",
                                            static_cast<int>(result.code));
        }

        count_++;
        step_length_ += (count_ %2) * step_increment_;
    }

bool ExploreState::isPointInBounds(double x, double y) const {
    return (x >= map_min_x_ && x <= map_max_x_ &&
            y >= map_min_y_ && y <= map_max_y_);
}

void ExploreState::clampGoalToBounds(double& x, double& y) {
    double original_x = x;
    double original_y = y;

    if (x < map_min_x_) x = static_cast<double>(map_min_x_);
    if (x > map_max_x_) x = static_cast<double>(map_max_x_);
    if (y < map_min_y_) y = static_cast<double>(map_min_y_);
    if (y > map_max_y_) y = static_cast<double>(map_max_y_);
    
    if (x != original_x || y != original_y) {
        RCLCPP_INFO(rclcpp::get_logger("ExploreState"),
                    "Clamped goal from (%f, %f) to (%f, %f)",
                    original_x, original_y, x, y);
    }
    
}

#pragma once

#include "States.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>

class ExploreState : public States {
    public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ExploreState();
    ~ExploreState() override;

    void update(SheepdogNode &context) override;
    States* transition(SheepdogNode &context) override;

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
    void initializeNavClient(SheepdogNode &context);
    void sendNextGoal(SheepdogNode &context);
    bool isPointInBounds(double x, double y) const;
    void clampGoalToBounds(double& x, double& y);
    void goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle);
    void goalResultCallback(const GoalHandleNav::WrappedResult& result);

};
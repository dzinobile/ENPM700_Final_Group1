#pragma once

#include "States.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

class SheepdogNode;

class EncircleState: public States {
    public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    EncircleState();
    ~EncircleState() override;

    void update(SheepdogNode &context) override;

    States* transition(SheepdogNode &context) override;

    private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;
    GoalHandleNav::SharedPtr current_goal_handle_;

    bool goal_sent_;
    bool goal_reached_;
    double offset_distance_;

    void initializeNavClient(SheepdogNode &context);
    void sendEncircleGoal(SheepdogNode &context);
    void goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle);
    void goalResultCallback(const GoalHandleNav::WrappedResult& result);    

};
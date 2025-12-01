/**
 * @file sheep_nav_trigger.cpp
 * @brief Monitors TF between base_footprint and sheep, triggers navigation to
 * the sheep, and saves the map when within a threshold distance.
 * @author Anvesh Som
 *
 * @copyright
 * Copyright (c) 2025 Sheepdog Project Team.
 * Licensed under the Apache License, Version 2.0. See the LICENSE file in the
 * project root for details.
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

/**
 * @class SheepNavTrigger
 * @brief ROS 2 node that monitors the distance between the robot and the sheep
 * and triggers Nav2 and map saving when close enough.
 *
 * The node periodically queries TF for the transforms map->base_footprint and
 * map->sheep, computes the distance between them in the map frame, and when the
 * distance is below a configurable threshold, sends a NavigateToPose goal to
 * the sheep position and calls the map_saver service to persist the current
 * map.
 */
class SheepNavTrigger : public rclcpp::Node {
 public:
  /// Alias for the NavigateToPose action type used by Nav2.
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  /// Alias for the goal handle associated with NavigateToPose.
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;
  /// Alias for the SaveMap service type used by nav2_map_server.
  using SaveMap = nav2_msgs::srv::SaveMap;

  /**
   * @brief Construct a new SheepNavTrigger node.
   *
   * Initializes TF2 buffer and listener, Nav2 action client, map saving service
   * client, and a periodic timer that performs TF lookup and distance checking.
   */
  SheepNavTrigger()
      : Node("sheep_nav_trigger"), trigger_distance_(2.0), triggered_(false) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    save_map_client_ = this->create_client<SaveMap>("/map_saver/save_map");

    timer_ = this->create_wall_timer(
        200ms, std::bind(&SheepNavTrigger::timerCallback, this));

    RCLCPP_INFO(get_logger(), "SheepNavTrigger node started.");
  }

 private:
  /**
   * @brief Timer callback that monitors the TFs and triggers navigation and map
   * saving.
   *
   * The callback performs the following steps:
   * - Looks up the transforms map->base_footprint and map->sheep.
   * - Computes the planar distance between base_footprint and sheep in the map
   * frame.
   * - If the distance is below @ref trigger_distance_, sends a Nav2 goal to the
   * sheep pose and calls the map_saver SaveMap service. After triggering once,
   * further callbacks return immediately due to the @ref triggered_ flag.
   */
  void timerCallback() {
    if (triggered_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_map_to_base;
    geometry_msgs::msg::TransformStamped tf_map_to_sheep;

    // 1) Get map->base_footprint
    try {
      tf_map_to_base =
          tf_buffer_->lookupTransform("map",                // target frame
                                      "base_footprint",     // source frame
                                      tf2::TimePointZero);  // latest available
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Could not get transform map->base_footprint: %s",
                           ex.what());
      return;
    }

    // 2) Get map->sheep
    try {
      tf_map_to_sheep =
          tf_buffer_->lookupTransform("map",                // target frame
                                      "sheep",              // source frame
                                      tf2::TimePointZero);  // latest available
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Could not get transform map->sheep: %s", ex.what());
      return;
    }

    // 3) Compute distance between base_footprint and sheep in map frame
    const double dx = tf_map_to_sheep.transform.translation.x -
                      tf_map_to_base.transform.translation.x;
    const double dy = tf_map_to_sheep.transform.translation.y -
                      tf_map_to_base.transform.translation.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    //   "Distance base_footprint->sheep in map frame = %.3f m", dist);

    if (dist > trigger_distance_) {
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Within %.2f m of sheep (dist = %.3f). Triggering Nav2 goal + "
                "map save.",
                trigger_distance_, dist);

    // 4) Send Nav2 goal to sheep pose (map frame)
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(get_logger(), "NavigateToPose action server not available");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = tf_map_to_sheep.transform.translation.x;
    goal.pose.pose.position.y = tf_map_to_sheep.transform.translation.y;
    goal.pose.pose.position.z = tf_map_to_sheep.transform.translation.z;
    goal.pose.pose.orientation = tf_map_to_sheep.transform.rotation;

    goal.behavior_tree = "";

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    /**
     * @brief Goal response callback for the NavigateToPose action.
     *
     * Logs whether the sent goal has been accepted or rejected by the Nav2
     * action server.
     *
     * @param handle Shared pointer to the goal handle, or nullptr if rejected.
     */
    send_goal_options.goal_response_callback =
        [this](GoalHandleNavigateToPose::SharedPtr handle) {
          if (!handle) {
            RCLCPP_ERROR(this->get_logger(),
                         "NavigateToPose goal was rejected");
          } else {
            RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted");
          }
        };

    /**
     * @brief Result callback for the NavigateToPose action.
     *
     * Logs whether navigation to the sheep pose succeeded or failed. The
     * trigger itself is one-shot and does not resend a goal based on this
     * result.
     *
     * @param result Wrapped result of the NavigateToPose action.
     */
    send_goal_options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Navigation to sheep SUCCESS");
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Navigation to sheep finished with code %d",
                        static_cast<int>(result.code));
          }
        };

    nav_client_->async_send_goal(goal, send_goal_options);

    // 5) Save map via map_saver
    if (!save_map_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(),
                  "SaveMap service '/map_saver/save_map' not available");
    } else {
      auto request = std::make_shared<SaveMap::Request>();
      request->map_topic = "map";
      request->map_url = "sheep_snapshot_map";
      request->image_format = "pgm";
      request->map_mode = "trinary";
      request->free_thresh = 0.25f;
      request->occupied_thresh = 0.65f;

      /**
       * @brief Callback for the SaveMap service response.
       *
       * Logs whether the map persistence operation reported success.
       *
       * @param future_resp Future containing the SaveMap response message.
       */
      auto future = save_map_client_->async_send_request(
          request, [this](rclcpp::Client<SaveMap>::SharedFuture future_resp) {
            auto resp = future_resp.get();
            if (resp->result) {
              RCLCPP_INFO(this->get_logger(),
                          "Map saved successfully by map_saver");
            } else {
              RCLCPP_WARN(this->get_logger(), "Map saver reported failure");
            }
          });
      (void)future;
    }

    triggered_ = true;
  }

  // --- members ---

  /**
   * @brief Distance threshold in meters for triggering navigation and map
   * saving.
   *
   * When the distance between the robot and the sheep in the map frame falls
   * below this value, the node sends the Nav2 goal and calls the map_saver
   * service.
   */
  double trigger_distance_;

  /**
   * @brief Flag indicating whether the trigger has already fired.
   *
   * Once set to true, the timer callback returns early to prevent repeated
   * navigation and map saving.
   */
  bool triggered_;

  /**
   * @brief Timer used to periodically check TF and evaluate the trigger
   * condition.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief TF2 buffer used to store and query transforms.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief TF2 transform listener that populates the TF buffer from /tf and
   * /tf_static topics.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief NavigateToPose action client used to send goals to the Nav2 stack.
   */
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  /**
   * @brief Client for the SaveMap service exposed by nav2_map_server.
   */
  rclcpp::Client<SaveMap>::SharedPtr save_map_client_;
};

/**
 * @brief Entry point for the SheepNavTrigger node.
 *
 * Initializes the ROS 2 system, instantiates the SheepNavTrigger node,
 * and starts spinning until shutdown is requested.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Exit code (0 on normal shutdown).
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SheepNavTrigger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

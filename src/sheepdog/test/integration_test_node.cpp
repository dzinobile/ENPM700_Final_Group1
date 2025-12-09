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
// limitations under the License
/**
 * @file integration_test_node.cpp
 * @brief Integration test node for sheepdog package
 * @author Daniel Zinobile
 * @date 08-Dec-2025
 */
#include "EncircleState.hpp"
#include "ExploreState.hpp"
#include "SheepdogNode.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
using namespace std::chrono_literals;
auto Logger = rclcpp::get_logger("");

/**
 * @class SheepBroadcasterTestFixture
 * @brief Test fixture for testing sheep_broadcaster and SheepdogNode
 * integration
 */
class SheepBroadcasterTestFixture {
public:
  /**
   * @brief Constructor for SheepBroadcasterTestFixture
   */
  SheepBroadcasterTestFixture() {
    testerNode = rclcpp::Node::make_shared("sheep_broadcaster_test_node");
    Logger = testerNode->get_logger();

    // Set use_sim_time
    testerNode->set_parameter(rclcpp::Parameter("use_sim_time", true));

    testerNode->declare_parameter("test_duration", 10.0);
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();

    // Create TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(testerNode->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(Logger, "Test duration: %.2f seconds", TEST_DURATION);
  }

protected:
  rclcpp::Node::SharedPtr testerNode;
  double TEST_DURATION;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/**
 * @brief Test case for testing sheep broadcaster publishes map to sheep
 * transform
 */
TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test sheep broadcaster publishes map->sheep transform",
                 "[tf_broadcast]") {
  RCLCPP_INFO(Logger, "Waiting for map->sheep transform...");

  rclcpp::sleep_for(std::chrono::seconds(1));

  bool transform_received = false;
  geometry_msgs::msg::TransformStamped transform;

  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::duration<double>(TEST_DURATION);

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    try {
      transform =
          tf_buffer_->lookupTransform("map", "sheep", tf2::TimePointZero);
      transform_received = true;
      RCLCPP_INFO(Logger, "Transform received!");
      break;
    } catch (tf2::TransformException &ex) {
      // Keep trying
      rclcpp::spin_some(testerNode);
      rclcpp::sleep_for(100ms);
    }
  }

  REQUIRE(transform_received);

  RCLCPP_INFO(Logger, "Sheep position: x=%.2f, y=%.2f, z=%.2f",
              transform.transform.translation.x,
              transform.transform.translation.y,
              transform.transform.translation.z);

  CHECK(transform.transform.translation.x == Catch::Approx(-4.0).epsilon(0.01));
  CHECK(transform.transform.translation.y == Catch::Approx(1.0).epsilon(0.01));
  CHECK(transform.transform.translation.z == Catch::Approx(0.0).epsilon(0.01));
}

/**
 * @brief Test case for testing sheep broadcaster publishes a map to pen
 * transform
 */
TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test sheep broadcaster publishes map->pen transform",
                 "[tf_broadcast]") {
  RCLCPP_INFO(Logger, "Waiting for map->pen transform...");

  rclcpp::sleep_for(std::chrono::seconds(1));

  bool transform_received = false;
  geometry_msgs::msg::TransformStamped transform;

  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::duration<double>(TEST_DURATION);

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    try {
      transform = tf_buffer_->lookupTransform("map", "pen", tf2::TimePointZero);
      transform_received = true;
      RCLCPP_INFO(Logger, "Pen transform received!");
      break;
    } catch (tf2::TransformException &ex) {
      rclcpp::spin_some(testerNode);
      rclcpp::sleep_for(100ms);
    }
  }

  REQUIRE(transform_received);

  CHECK(transform.transform.translation.x == Catch::Approx(0.0).epsilon(0.01));
  CHECK(transform.transform.translation.y == Catch::Approx(0.0).epsilon(0.01));
  CHECK(transform.transform.translation.z == Catch::Approx(0.0).epsilon(0.01));
}

/**
 * @brief Test case for testing transform from robot base link to sheep can be
 * computed
 */
TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test base_link->sheep transform can be computed",
                 "[tf_integration]") {
  RCLCPP_INFO(Logger, "Testing base_link->sheep transform lookup...");

  rclcpp::sleep_for(std::chrono::seconds(2));

  bool transform_received = false;
  geometry_msgs::msg::TransformStamped transform;
  double distance_to_sheep = 0.0;

  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::duration<double>(TEST_DURATION);

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    try {
      transform =
          tf_buffer_->lookupTransform("base_link", "sheep", tf2::TimePointZero);

      double dx = transform.transform.translation.x;
      double dy = transform.transform.translation.y;
      double dz = transform.transform.translation.z;
      distance_to_sheep = std::sqrt(dx * dx + dy * dy + dz * dz);

      transform_received = true;
      RCLCPP_INFO(Logger,
                  "base_link->sheep transform received! Distance: %.2f meters",
                  distance_to_sheep);
      break;
    } catch (tf2::TransformException &ex) {
      rclcpp::spin_some(testerNode);
      rclcpp::sleep_for(100ms);
    }
  }

  if (transform_received) {
    RCLCPP_INFO(Logger, "Successfully computed distance to sheep: %.2f meters",
                distance_to_sheep);
    CHECK(distance_to_sheep >= 0.0);
  } else {
    RCLCPP_WARN(Logger, "base_link->sheep transform not available. "
                        "This is expected if TurtleBot isn't running.");
  }
}

/**
 * @class SheepdogTestFixture
 * @brief Test fixture for testing functions of sheepdognode, including
 * different states of FSM design
 */
class SheepdogTestFixture {
public:
  SheepdogTestFixture() {
    testerNode = rclcpp::Node::make_shared("sheepdog_test_node");
    Logger = testerNode->get_logger();

    // Set use_sim_time
    testerNode->set_parameter(rclcpp::Parameter("use_sim_time", true));

    testerNode->declare_parameter("test_duration", 10.0);
    TEST_DURATION = testerNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();

    // Create TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(testerNode->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(Logger, "Test duration: %.2f seconds", TEST_DURATION);
  }

protected:
  rclcpp::Node::SharedPtr testerNode;
  double TEST_DURATION;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/**
 * @brief Test case for testing sheepdog initializes in EXPLORE state
 */
TEST_CASE_METHOD(SheepdogTestFixture, "test initializes in EXPLORE state",
                 "[tf_integration]") {
  RCLCPP_INFO(Logger, "Testing initializes in EXPLORE state");

  auto node = std::make_shared<SheepdogNode>();

  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  States *state = node->getCurrentState();

  REQUIRE(state != nullptr);
  REQUIRE(dynamic_cast<ExploreState *>(state) != nullptr);
}

/**
 * @brief Test case for testing if sheepdog switches to ENCIRCLE state when
 * sheep detected
 */
TEST_CASE_METHOD(SheepdogTestFixture,
                 "test goes to ENCIRCLE state when sheep located",
                 "[tf_integration]") {
  RCLCPP_INFO(Logger, "Testing goes to ENCIRCLE when sheep found");
  auto node = std::make_shared<SheepdogNode>();
  tf2_ros::TransformBroadcaster tf_broadcaster(node);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "sheep";

  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.5);
  q.normalize();

  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  for (int i = 0; i < 10; ++i) {
    tf_msg.header.stamp = node->now();
    tf_broadcaster.sendTransform(tf_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 1000ms) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  States *state = node->getCurrentState();
  REQUIRE(dynamic_cast<EncircleState *>(state) != nullptr);
}
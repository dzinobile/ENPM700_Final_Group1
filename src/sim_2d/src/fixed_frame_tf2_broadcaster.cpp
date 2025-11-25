// Copyright 2021 Open Source Robotics Foundation, Inc.
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
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  FixedFrameBroadcaster()
  : Node("fixed_frame_tf2_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  int num_turtles_ = 9;
  void broadcast_timer_callback()
  {
    geometry_msgs::msg::TransformStamped turtle1_tf;
    try{
      turtle1_tf = tf_buffer_->lookupTransform(
        "world",
        "turtle1",
        tf2::TimePointZero
      );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "could not get transform");
      return;
    }


    std::vector<std::string> others;
    for (int i = 2; i < num_turtles_ + 1; i++) {
      std::string t_name = "turtle" + std::to_string(i);
      others.push_back(t_name);
    }


    double range = 5.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double carrot_dist = 2.0;
    int count = 0;

    for (const auto &name : others) {
      geometry_msgs::msg::TransformStamped other_tf;
      try {
        other_tf = tf_buffer_->lookupTransform(
          "world", name, tf2::TimePointZero
        );
      } catch (...) {
        continue;
      }
      
      double dx = other_tf.transform.translation.x - turtle1_tf.transform.translation.x;
      double dy = other_tf.transform.translation.y - turtle1_tf.transform.translation.y;
      double dist = sqrt(pow(dx,2)+pow(dy,2));

      if (dist < range) {
        sum_x += other_tf.transform.translation.x;
        sum_y += other_tf.transform.translation.y;
        count++;
      }

    }

    double carrot_x = turtle1_tf.transform.translation.x;
    double carrot_y = turtle1_tf.transform.translation.y;

    if (count > 0) {
      double avg_x = sum_x / count;
      double avg_y = sum_y / count;

      double rx = turtle1_tf.transform.translation.x - avg_x;
      double ry = turtle1_tf.transform.translation.y - avg_y;

      double mag = sqrt(pow(rx,2)+pow(ry,2));
      if (mag > 0.001) {
        rx /= mag;
        ry /= mag;
      }

      carrot_x = turtle1_tf.transform.translation.x + rx * carrot_dist;
      carrot_y = turtle1_tf.transform.translation.y + ry * carrot_dist;

    }


    geometry_msgs::msg::TransformStamped t1;
    t1.header.stamp = this->get_clock()->now();
    t1.header.frame_id = "world";
    t1.child_frame_id = "carrot1";
    t1.transform.translation.x = carrot_x;
    t1.transform.translation.y = carrot_y;
    t1.transform.translation.z = 0.0;
    t1.transform.rotation.x = 0.0;
    t1.transform.rotation.y = 0.0;
    t1.transform.rotation.z = 0.0;
    t1.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t1);

    std::vector<double> x_offset = {
      2.0, 0.0, 0.0, -2.0, 2/sqrt(2.0), -2/sqrt(2.0),
      2/sqrt(2.0), -2/sqrt(2.0) 
    };

    std::vector<double> y_offset = {
      0.0, 2.0, -2.0, 0.0, 2/sqrt(2.0), -2/sqrt(2.0),
      -2/sqrt(2.0), 2/sqrt(2.0)

    };

    for (int i = 2; i < num_turtles_+1; i++){
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world";
      t.child_frame_id = "carrot"+std::to_string(i);
      t.transform.translation.x = turtle1_tf.transform.translation.x + x_offset[i-2];
      t.transform.translation.y = turtle1_tf.transform.translation.y + y_offset[i-2];
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(t);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

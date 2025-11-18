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
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    N_(9)
  {
    // Declare and acquire `target_frame` parameter
    spawned_.resize(N_+1, false);
    target_frames_.resize(N_ + 1);
    publishers_.resize(N_ + 1);

    for (int i = 1; i <= N_; i++) {
      std::string pname = "target_frame" + std::to_string(i);
      target_frames_[i] = 
      this->declare_parameter<std::string>(pname, "turtle1");
    }
    
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    for (int i = 1; i <= N_; i++) {
      std::string topic = "turtle" + std::to_string(i) + "/cmd_vel";
      publishers_[i] = 
      this->create_publisher<geometry_msgs::msg::Twist>(topic, 1);
    }

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, [this]() {return this->on_timer();});
  }

private:
  void on_timer() {
    if (!spawner_->service_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "spawn service not ready");
      return;
    }

    for (int i = 2; i <= N_; i++) {
      if (!spawned_[i]) {
        auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
        req->x = i - 1;
        req->y = 1.0;
        req->theta = 0.0;
        req->name = "turtle" + std::to_string(i);

        auto cb = [this, i](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
          if (future.get()->name == ("turtle" + std::to_string(i))) {
            RCLCPP_INFO(this->get_logger(), "Spawned turtle%d", i);
            spawned_[i] = true;
          }
        };

        spawner_->async_send_request(req, cb);
        return;

      }
    }


    for (int i = 1; i <= N_; i++) {
      std::string child = "turtle" + std::to_string(i);
      control_turtle(child, publishers_[i], target_frames_[i], i == 1);
    }
    
  }

  void control_turtle(const std::string &child_frame,
                      const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &pub,
                      const std::string &target_frame,
                      bool is_leader)
  {
    geometry_msgs::msg::TransformStamped t;

    try{
      t = tf_buffer_->lookupTransform(
        child_frame,
        target_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "could not transform %s -> %s", child_frame.c_str(), target_frame.c_str());
      return;
    }

    geometry_msgs::msg::Twist msg;

    msg.angular.z = atan2(t.transform.translation.y,
                          t.transform.translation.x);
    msg.linear.x = 0.5 * sqrt(
      pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
    
    if (!is_leader) {
      msg.linear.x *= 2.5;
    } 

    if (msg.linear.x < 0.5) {
      msg.linear.x = 0.0;
    }
    
    pub->publish(msg);
  }   
  
  int N_;
  std::vector<bool> spawned_;
  std::vector<std::string> target_frames_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}

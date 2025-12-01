#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MapToSheepBroadcaster : public rclcpp::Node
{
public:
  MapToSheepBroadcaster()
  : Node("sheep_broadcaster")
  {
    // Create a TransformBroadcaster object
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer to publish at 10 Hz
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MapToSheepBroadcaster::on_timer, this));
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    // Timestamp and frame ids
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "map";     // parent frame
    tf_msg.child_frame_id = "sheep";    // child frame

    // Example translation (change as needed)
    tf_msg.transform.translation.x = 3.0;
    tf_msg.transform.translation.y = 1.5;
    tf_msg.transform.translation.z = 0.0;

    // Example rotation: roll = 0, pitch = 0, yaw = 0.5 rad
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.5);
    q.normalize();

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    // Publish the transform
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapToSheepBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
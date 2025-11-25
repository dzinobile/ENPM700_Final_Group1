#include "robot_broadcaster.hpp"

FramePublisher::FramePublisher() : rclcpp::Node("frame_publisher") {
    robot_name_ = this->declare_parameter<std::string>("robotname", "robot");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::ostringstream stream;
    stream << "/" << robot_name_.c_str() << "/diffdrive_controller/odom";
    std::string topic_name = stream.str();

    auto handle_robot_pose = [this](const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = robot_name_.c_str();

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        t.transform.rotation.x = msg->pose.pose.orientation.x;
        t.transform.rotation.y = msg->pose.pose.orientation.y;
        t.transform.rotation.z = msg->pose.pose.orientation.z;
        t.transform.rotation.w = msg->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(t);
    };

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name, 10, handle_robot_pose
    );


}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
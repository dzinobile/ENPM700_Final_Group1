#include "listener.hpp"

using namespace std::chrono_literals;
FrameListener::FrameListener() : rclcpp::Node("frame_listener") {
    num_robots_ = 10;
    target_frames_.resize(num_robots_ + 1);
    publishers_.resize(num_robots_ + 1);

    for (int i = 0; i < num_robots_ + 1; i++) {
        std::string pname = "target_frame" + std::to_string(i);
        target_frames_[i] = 
            this->declare_parameter<std::string>(pname, "robot0");
    }

    tf_buffer_ = 
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = 
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    for (int i = 0; i < num_robots_ + 1; i++) {
        std::string topic = "/robot" + std::to_string(i) + "/diffdrive_controller/cmd_vel_unstamped";
        publishers_[i] = 
            this->create_publisher<geometry_msgs::msg::Twist>(topic, 1);
    }

    timer_ = this->create_wall_timer(
        1s, [this]() {return this->onTimer();}
    );
}

void FrameListener::onTimer(){
    for (int i = 0; i < num_robots_ + 1; i++) {
        std::string child = "robot" + std::to_string(i);
        controlRobot(child, publishers_[i], target_frames_[i], i == 0);
    }
    
}

void FrameListener::controlRobot(const std::string &child_frame,
                    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &pub,
                    const std::string &target_frame,
                    bool is_leader) 
    {
    geometry_msgs::msg::TransformStamped t;

    try{
        t = tf_buffer_->lookupTransform(
            child_frame,
            target_frame,
            tf2::TimePointZero
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "could not transform %s -> %s", child_frame.c_str(), target_frame.c_str());
        return;
    }
    geometry_msgs::msg::Twist msg;

    // Calculate angle to target
    double angle_to_target = atan2(t.transform.translation.y,
                                    t.transform.translation.x);

    // Calculate distance to target
    double distance = sqrt(pow(t.transform.translation.x, 2) + 
                        pow(t.transform.translation.y, 2));

    // Proportional control for angular velocity
    // This makes the robot turn towards the target smoothly
    msg.angular.z = 4.0 * angle_to_target;  // Tune this gain (try 2.0-6.0)

    // Only move forward if roughly facing the target
    if (abs(angle_to_target) < 0.5) {  // Within ~30 degrees
        msg.linear.x = 0.5 * distance;
    } else {
        msg.linear.x = 0.1 * distance;  // Move slower while turning
    }

    // Optional: limit velocities to your max speeds
    msg.linear.x = std::min(msg.linear.x, 0.66);   // Your max linear speed
    msg.angular.z = std::clamp(msg.angular.z, -2.0, 2.0);  // Reasonable angular limit

    if (!is_leader) {
      msg.linear.x *= 1.5;
    } 

    if (msg.linear.x < 0.5) {
      msg.linear.x = 0.0;
    }

    pub->publish(msg);

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
}
#include "simple_publisher.hpp"

using namespace std::chrono_literals;


SimplePublisher::SimplePublisher() :  rclcpp::Node("simple_publisher"){
    num_bots_ = 10;
    for (int i = 0; i < num_bots_+1; i++){
        publishers_.push_back(this->create_publisher<geometry_msgs::msg::Twist>("robot" + std::to_string(i) + "/diffdrive_controller/cmd_vel_unstamped", 10));
    }
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SimplePublisher::timerCallback, this));
}

void SimplePublisher::timerCallback(){
    auto message = geometry_msgs::msg::Twist();
    for (int i = 0; i< num_bots_+1; i++) {
        message.linear.x = 0.05 * i;
        message.angular.z = 0.0;
        RCLCPP_INFO_STREAM(this->get_logger(),"publishing to bot" + std::to_string(i) + " speed: " + std::to_string(message.linear.x) );
        publishers_[i]->publish(message);


    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
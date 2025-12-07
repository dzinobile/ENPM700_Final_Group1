#include <rclcpp/rclcpp.hpp>
#include "SheepdogNode.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SheepdogNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
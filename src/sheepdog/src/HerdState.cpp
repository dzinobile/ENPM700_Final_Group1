#include "HerdState.hpp"
#include "SheepdogNode.hpp"

HerdState::HerdState() {}

HerdState::~HerdState(){}

void HerdState::update(SheepdogNode &context) {
    RCLCPP_INFO_THROTTLE(context.get_logger(),
                        *context.get_clock(),
                        5000,
                        "HERD: dummy state active");
}

States* HerdState::transition(SheepdogNode &context) {
    return nullptr;
}
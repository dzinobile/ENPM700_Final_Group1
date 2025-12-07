#include "EncircleState.hpp"
#include "SheepdogNode.hpp"

EncircleState::EncircleState() {}

EncircleState::~EncircleState(){}

void EncircleState::update(SheepdogNode &context) {
    RCLCPP_INFO_THROTTLE(context.get_logger(),
                        *context.get_clock(),
                        5000,
                        "ENCIRCLE: dummy state active");
}

States* EncircleState::transition(SheepdogNode &context) {
    return nullptr;
}
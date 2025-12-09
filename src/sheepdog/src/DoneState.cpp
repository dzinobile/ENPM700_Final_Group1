#include "DoneState.hpp"
#include "SheepdogNode.hpp"

DoneState::DoneState() {}

DoneState::~DoneState() {}

void DoneState::update(SheepdogNode &context) {
  RCLCPP_INFO_THROTTLE(context.get_logger(), *context.get_clock(), 5000,
                       "DONE");
}

States *DoneState::transition(SheepdogNode &context) { return nullptr; }
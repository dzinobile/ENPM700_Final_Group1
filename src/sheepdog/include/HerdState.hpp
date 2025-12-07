#pragma once

#include "States.hpp"

class SheepdogNode;

class HerdState: public States {
    HerdState();
    ~HerdState() override;

    void update(SheepdogNode &context) override;

    States* transition(SheepdogNode &context) override;
};
#pragma once

#include "States.hpp"

class SheepdogNode;

class EncircleState: public States {
    public:
    EncircleState();
    ~EncircleState() override;

    void update(SheepdogNode &context) override;

    States* transition(SheepdogNode &context) override;
};
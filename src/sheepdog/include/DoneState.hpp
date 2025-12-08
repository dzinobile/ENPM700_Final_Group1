#pragma once

#include "States.hpp"

class SheepdogNode;

class DoneState: public States {
    public:
    DoneState();
    ~DoneState() override;

    void update(SheepdogNode &context) override;

    States* transition(SheepdogNode &context) override;
};
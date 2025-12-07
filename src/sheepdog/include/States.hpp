#pragma once

#include <memory>

class SheepdogNode;

class States {
    public:
    States();
    virtual ~States();
    virtual void update(SheepdogNode &context) = 0;
    virtual States *transition(SheepdogNode &context) = 0;
};


#pragma once

namespace simple_flight {

class IUpdatable {
public:
    virtual void reset() = 0;
    virtual void update() = 0;
};

}
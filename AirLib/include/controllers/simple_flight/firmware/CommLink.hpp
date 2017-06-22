#pragma once

#include <cstdint>


namespace simple_flight {

class CommLink {
public:
    virtual void reset() = 0;
    virtual void update() = 0;
    virtual void log(const char* message, int32_t error_level) = 0;
};


} //namespace
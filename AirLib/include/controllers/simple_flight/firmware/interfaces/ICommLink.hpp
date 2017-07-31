#pragma once

#include <cstdint>

namespace simple_flight {

class ICommLink : public IUpdatable {
public:
    virtual void log(const char* message, int32_t error_level) = 0;
};

} //namespace
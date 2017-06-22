#pragma once

#include <cstdint>


namespace ros_flight {

class CommLink {
public:
    virtual void init() = 0;
    virtual void update() = 0;
    virtual void set_sys_id(int32_t sys_id) = 0;
    virtual void set_streaming_rate(uint16_t param_id, int32_t rate) = 0;
    virtual void notify_param_change(uint16_t param_id, int32_t value) = 0;
    virtual void log_message(const char* message, uint8_t error_level) = 0;
    virtual void notify_controller_updated() = 0;
};


} //namespace
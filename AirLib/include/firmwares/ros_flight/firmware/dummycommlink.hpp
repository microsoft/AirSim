#pragma once

#include "commlink.hpp"


namespace ros_flight {

class DummyCommLink : public CommLink {
public:
    virtual void init() override {}
    virtual void update() override {}
    virtual void set_sys_id(int32_t sys_id) override {}
    virtual void set_streaming_rate(uint16_t param_id, int32_t rate) override {}
    virtual void notify_param_change(uint16_t param_id, int32_t value) override {}
    virtual void log_message(const char* message, uint8_t error_level) override {}
    virtual void notify_controller_updated() {}
};


} //namespace
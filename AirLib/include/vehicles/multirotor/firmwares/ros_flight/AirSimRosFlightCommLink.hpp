// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimRosFlightCommLink_hpp
#define msr_airlib_AirSimRosFlightCommLink_hpp

#include <exception>
#include "firmware/commlink.hpp"
#include "common/Common.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "sensors/SensorCollection.hpp"


namespace msr { namespace airlib {


class AirSimRosFlightCommLink : public ros_flight::CommLink {
public: // derived class specific methods
    void getStatusMessages(std::vector<std::string>& messages)
    {
        if (messages_.size() > 0) {
            messages.insert(messages.end(), messages_.begin(), messages_.end());
            messages_.clear();
        }
    }

    virtual ~AirSimRosFlightCommLink() = default;

public: // implement CommLink interface
    virtual void init()
    {
        messages_.clear();
    }

    virtual void update()
    {
    }

    virtual void set_sys_id(int32_t sys_id)
    {
        unused(sys_id);
    }

    virtual void set_streaming_rate(uint16_t param_id, int32_t rate)
    {
        unused(param_id);
        unused(rate);
    }

    virtual void notify_param_change(uint16_t param_id, int32_t value)
    {
        unused(param_id);
        unused(value);
    }

    virtual void log_message(const char* message, uint8_t error_level)
    {
        unused(error_level);
        messages_.push_back(std::string(message));
    }

    virtual void notify_controller_updated()
    {
    }

private:
    std::vector<std::string> messages_;

};


}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightCommLink_hpp
#define msr_airlib_AirSimSimpleFlightCommLink_hpp

#include <exception>
#include "firmware/interfaces/ICommLink.hpp"
#include "common/Common.hpp"


namespace msr { namespace airlib {


class AirSimSimpleFlightCommLink : public simple_flight::ICommLink {
public: // derived class specific methods
    void getStatusMessages(std::vector<std::string>& messages)
    {
        if (messages_.size() > 0) {
            messages.insert(messages.end(), messages_.begin(), messages_.end());
            messages_.clear();
        }
    }

public: // implement CommLink interface
    virtual void reset()
    {
        messages_.clear();
    }

    virtual void update()
    {
    }

    virtual void log(const char* message, int32_t error_level)
    {
        unused(error_level);
        messages_.push_back(std::string(message));
    }

private:
    std::vector<std::string> messages_;

};


}} //namespace
#endif

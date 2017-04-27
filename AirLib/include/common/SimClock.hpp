// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_SimClock_hpp
#define airsim_core_SimClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"

namespace msr { namespace airlib {

class SimClock : public ClockBase {
public:
    virtual TTimePoint nowNanos() override
    {
        return Utils::getTimeSinceEpochNanos();
    }
};

}} //namespace
#endif

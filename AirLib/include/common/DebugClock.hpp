// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_DebugClock_hpp
#define airsim_core_DebugClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"

namespace msr { namespace airlib {

class DebugClock : public ClockBase {
public:
    DebugClock(TTimePoint start = 1000, TTimePoint step = 20E6)
        : current_(start), step_(step)
    {
        current_ = start;
    }

    void add(TTimePoint amount)
    {
        current_ += amount;
    }

    void stepNext()
    {
        current_ += step_;
    }

    virtual TTimePoint nowNanos() override
    {
        return current_;
    }

    virtual TTimeDelta fromWallDelta(TTimeDelta dt) override
    {
        return dt;
    }
    virtual TTimeDelta toWallDelta(TTimeDelta dt)  override
    {
        return dt;
    }


private:
    TTimePoint current_, step_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_SimClock_hpp
#define airsim_core_SimClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"

namespace msr { namespace airlib {

class SimClock : public ClockBase {
public:
    SimClock(double scale = 1, TTimeDelta latency = 0)
        : scale_(scale), latency_(latency)
    {
        offset_ = latency * (scale_ - 1);
    }

    virtual TTimePoint nowNanos() override
    {
        if (offset_ == 0 && scale_ == 1) //optimized normal route
            return Utils::getTimeSinceEpochNanos();
        else {
            /*
                Apply scaling and latency.

                Time point is nanoseconds from some reference r. If latency = 0 then r = 0 .
                scaled time point is then given by (r + ((now - r) / scale)).
                This becomes (r*(s-1) + now)/scale or (offset + now / scale).
            */
            return static_cast<TTimePoint>((Utils::getTimeSinceEpochNanos() + offset_) / scale_);
        }
    }

    virtual TTimeDelta fromWallDelta(TTimeDelta dt) override
    {
        return dt * scale_;
    }
    virtual TTimeDelta toWallDelta(TTimeDelta dt)  override
    {
        return dt / scale_;
    }


private:
    double scale_;
    TTimeDelta latency_;
    double offset_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ScalableClock_hpp
#define airsim_core_ScalableClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"

namespace msr { namespace airlib {

//ScalableClock is a clock that can be scaled (i.e. slowed down or speeded up)
class ScalableClock : public ClockBase {
public:
    //scale > 1 slows down the clock, < 1 speeds up the clock
    ScalableClock(double scale = 1, TTimeDelta latency = 0)
        : scale_(scale), latency_(latency)
    {
        offset_ = latency * (scale_ - 1);
        unused(latency_);
    }

    virtual TTimePoint nowNanos() const override
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

    virtual void sleep_for(TTimeDelta dt) override
    {
        //for intervals > 2ms just sleep otherwise do spilling otherwise delay won't be accurate
        if (dt > 2.0 / 1000) {
            TTimeDelta dt_scaled = fromWallDelta(dt);
            std::this_thread::sleep_for(std::chrono::duration<double>(dt_scaled));
        }
        else
            ClockBase::sleep_for(dt);
    }

protected:
    //converts time interval for wall clock to current clock
    //For example, if implementation is scaled clock simulating 5X spped then below
    //will retun dt*5. This functions are required to translate time to operating system
    //which only has concept of wall clock. For example, to make thread sleep for specific duration.

    //wall clock to sim clock
    virtual TTimeDelta fromWallDelta(TTimeDelta dt) const
    {
        return dt * scale_;
    }
    //sim clock to wall clock
    virtual TTimeDelta toWallDelta(TTimeDelta dt) const 
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

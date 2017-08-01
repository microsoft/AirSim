// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_DebugClock_hpp
#define airsim_core_DebugClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"
#include <atomic>

namespace msr { namespace airlib {

class DebugClock : public ClockBase {
public:
    //Debug clock allows to advance the clock manually in arbitrary way
    //TTimePoint is nano seconds passed since some fixed point in time
    //step is how much we would advance the clock by default when calling step()
    //by default we advance by 20ms
    DebugClock(TTimeDelta step = 20E-3f, TTimePoint start = 1000)
        : current_(start), step_(step)
    {
        current_ = start;
    }

    TTimePoint step(TTimeDelta amount)
    {
        current_ = addTo(current_, amount);
        return current_;
    }

    TTimePoint step()
    {
        current_ = addTo(current_, step_);
        return current_;
    }

    virtual TTimePoint nowNanos() const override
    {
        return current_;
    }

    //convert wall clock interval to this clock interval
    //below functions should normally be used in things like thread.sleep 
    //which usually requires wall time (i.e. clock used by system)
    virtual TTimeDelta fromWallDelta(TTimeDelta dt) const override
    {
        return dt;
    }
    virtual TTimeDelta toWallDelta(TTimeDelta dt) const  override
    {
        return dt;
    }


private:
    std::atomic<TTimePoint> current_;
    TTimeDelta step_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_Waiter_hpp
#define air_Waiter_hpp

#include <chrono>
#include <iostream>
#include "common/Common.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/ClockFactory.hpp"
#include "common/common_utils/WorkerThread.hpp"

using namespace msr::airlib;

namespace msr { namespace airlib {

class Waiter {
private:
    ClockBase* clock_ = ClockFactory::get();

    TTimePoint proc_start_;
    TTimePoint loop_start_;

    TTimeDelta sleep_duration_, timeout_duration_;
public:
    Waiter(TTimeDelta sleep_duration_seconds, TTimeDelta timeout_duration = std::numeric_limits<TTimeDelta>::max())
        : sleep_duration_(sleep_duration_seconds), timeout_duration_(timeout_duration)
    {
        proc_start_ = loop_start_ = clock_->nowNanos();
    }

    virtual bool sleep(CancelableBase& cancelable_action)
    {
        // Sleeps for the time needed to get current running time up to the requested sleep_duration_.
        // So this can be used to "throttle" any loop to check something every sleep_duration_ seconds.
        TTimeDelta running_time = clock_->elapsedSince(loop_start_);
        double remaining = sleep_duration_ - running_time;
        bool completed = cancelable_action.sleep(clock_->toWallDelta(remaining));
        loop_start_ = clock_->nowNanos();
        return completed;
    }

    void resetSleep()
    {
    	loop_start_ = clock_->nowNanos();
    }
    void resetTimeout()
    {
    	proc_start_ = clock_->nowNanos();
    }

    bool is_timeout() const
    {
    	return clock_->elapsedSince(proc_start_) >= timeout_duration_;
    }
};

}} //namespace
#endif

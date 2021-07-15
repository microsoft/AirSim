// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_Waiter_hpp
#define air_Waiter_hpp

#include <chrono>
#include <iostream>
#include "common/Common.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/ClockFactory.hpp"
#include "common/CancelToken.hpp"

namespace msr
{
namespace airlib
{

    class Waiter
    {
    public:
        Waiter(TTimeDelta sleep_duration_seconds, TTimeDelta timeout_sec, CancelToken& cancelable_action)
            : sleep_duration_(sleep_duration_seconds), timeout_sec_(timeout_sec), cancelable_action_(cancelable_action), is_complete_(false)
        {
            proc_start_ = loop_start_ = clock()->nowNanos();
        }

        bool sleep()
        {
            // Sleeps for the time needed to get current running time up to the requested sleep_duration_.
            // So this can be used to "throttle" any loop to check something every sleep_duration_ seconds.

            if (isComplete())
                throw std::domain_error("Process was already complete. This instance of Waiter shouldn't be reused!");
            if (isTimeout())
                return false;

            //measure time spent since last iteration
            TTimeDelta running_time = clock()->elapsedSince(loop_start_);
            double remaining = sleep_duration_ - running_time;
            bool done = cancelable_action_.sleep(remaining);
            loop_start_ = clock()->nowNanos();
            return done;
        }

        //call this mark process as complete
        void complete()
        {
            is_complete_ = true;
        }

        bool isComplete() const
        {
            return is_complete_;
        }

        bool isTimeout() const
        {
            if (isComplete())
                return false;
            else
                return clock()->elapsedSince(proc_start_) >= timeout_sec_;
        }

    private:
        TTimeDelta sleep_duration_, timeout_sec_;
        CancelToken& cancelable_action_;
        bool is_complete_; //each waiter should maintain its own complete status

        TTimePoint proc_start_;
        TTimePoint loop_start_;

        static ClockBase* clock()
        {
            return ClockFactory::get();
        }
    };
}
} //namespace
#endif

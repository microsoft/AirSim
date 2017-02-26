// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_Waiter_hpp
#define air_Waiter_hpp

#include <chrono>
#include <iostream>
#include "common/Common.hpp"
#include "common/common_utils/Utils.hpp"

namespace msr { namespace airlib {

class CancelableBase {
public:
    virtual bool isCancelled() = 0;
    virtual void cancelAllTasks() = 0;
    virtual bool sleep(double secs)
    {
        //We can pass duration directly to sleep_for however it is known that on 
        //some systems, sleep_for makes system call anyway even if passed duration 
        //is <= 0. This can cause 50us of delay due to context switch.
        if (isCancelled()) {
            Utils::logMessage("Sleep was prempted");
            return false;
        }

        if (secs > 0)
            std::this_thread::sleep_for(std::chrono::duration<double>(secs));
        else
            Utils::logMessage("Missed sleep: %f ms", secs*1000);

        return !isCancelled();
    }

    virtual ~CancelableBase() = default;
};

class Waiter {
private:
    typedef std::chrono::steady_clock steady_clock;

    std::chrono::time_point<std::chrono::steady_clock> proc_start_ = steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> loop_start_ = proc_start_;         

    std::chrono::duration<double> sleep_duration_, timeout_duration_;
public:
    Waiter(double sleep_duration_seconds, double timeout_duration = std::numeric_limits<float>::max())
        : sleep_duration_(sleep_duration_seconds), timeout_duration_(timeout_duration)
    {}

    virtual bool sleep(CancelableBase& cancelable_action)
    {
        // Sleeps for the time needed to get current running time up to the requested sleep_duration_.
        // So this can be used to "throttle" any loop to check something every sleep_duration_ seconds.
        auto running_time = std::chrono::duration<double>(steady_clock::now() - loop_start_);
        double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(sleep_duration_ - running_time).count();
        bool completed = cancelable_action.sleep(seconds);
        loop_start_ = steady_clock::now();
        return completed;
    }

    void resetSleep()
    {
    	loop_start_ = steady_clock::now();
    }
    void resetTimeout()
    {
    	proc_start_ = steady_clock::now();
    }

    bool is_timeout() const
    {
    	bool y = std::chrono::duration_cast<std::chrono::duration<double>>
            (steady_clock::now() - proc_start_).count() >= timeout_duration_.count();

        return y;
    }
};

}} //namespace
#endif

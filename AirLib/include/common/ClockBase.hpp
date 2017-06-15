// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ClockBase_hpp
#define airsim_core_ClockBase_hpp

#include <thread>
#include <chrono>
#include "Common.hpp"

namespace msr { namespace airlib {

class ClockBase {
public:
    //returns value indicating nanoseconds elapsed since some reference timepoint in history
    //typically nanoseconds from Unix epoch
    virtual TTimePoint nowNanos() = 0;
    //converts time interval for wall clock to current clock
    //For example, if implementation is scaled clock simulating 5X spped then below
    //will retun dt*5. This functions are required to translate time to operating system
    //which only has concept of wall clock. For example, to make thread sleep for specific duration.
    virtual TTimeDelta fromWallDelta(TTimeDelta dt) = 0;
    virtual TTimeDelta toWallDelta(TTimeDelta dt) = 0;


    TTimeDelta elapsedSince(TTimePoint since)
    {
        return elapsedBetween(nowNanos(), since);
    }
    static TTimeDelta elapsedBetween(TTimePoint second, TTimePoint first)
    {
        return (second - first) / 1.0E9;
    }
    TTimeDelta updateSince(TTimePoint& since)
    {
        TTimePoint cur = nowNanos();
        double elapsed = elapsedBetween(cur, since);
        since = cur;
        return elapsed;
    }

    void sleep_for(TTimeDelta wall_clock_dt)
    {
        TTimeDelta dt = fromWallDelta(wall_clock_dt);

        if (dt <= 0)
            return;

        //for intervals > 2ms just sleep otherwise do spilling otherwise delay won't be accurate
        if (dt > 2.0/1000)  
            std::this_thread::sleep_for(duration<double>(dt));
        else {
            static constexpr std::chrono::duration<double> MinSleepDuration(0);
            clock::time_point start = clock::now();
            //spin wait
            while (duration<double>(clock::now() - start).count() < dt) {
                std::this_thread::sleep_for(MinSleepDuration);
            }
        }
    }

private:
    typedef std::chrono::high_resolution_clock clock;
    template <typename T>
    using duration = std::chrono::duration<T>;
};

}} //namespace
#endif

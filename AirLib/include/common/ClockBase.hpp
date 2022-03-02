// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ClockBase_hpp
#define airsim_core_ClockBase_hpp

#include <thread>
#include <chrono>
#include "Common.hpp"

namespace msr
{
namespace airlib
{

    class ClockBase
    {
    public:
        //returns value indicating nanoseconds elapsed since some reference timepoint in history
        //typically nanoseconds from Unix epoch
        virtual TTimePoint nowNanos() const = 0;
        virtual TTimePoint getStart() const = 0;

        ClockBase()
        {
            wall_clock_start_ = Utils::getTimeSinceEpochNanos();
        }

        virtual ~ClockBase() = default;

        TTimeDelta elapsedSince(TTimePoint since) const
        {
            return elapsedBetween(nowNanos(), since);
        }
        static TTimeDelta elapsedBetween(TTimePoint second, TTimePoint first)
        {
            return (second - first) / 1.0E9;
        }
        TTimePoint addTo(TTimePoint t, TTimeDelta dt)
        {
            return static_cast<TTimePoint>(t + dt * 1.0E9);
        }
        TTimeDelta updateSince(TTimePoint& since) const
        {
            TTimePoint cur = nowNanos();
            TTimeDelta elapsed = elapsedBetween(cur, since);
            since = cur;
            return elapsed;
        }

        virtual TTimePoint stepBy(TTimeDelta amount)
        {
            unused(amount);
            return step();
        }

        virtual TTimePoint step()
        {
            //by default step doesn't do anything
            //for steppeble clock, this would advance to next tick
            //for wall clocks, step() is no-op
            ++step_count_;

            return nowNanos();
        }

        uint64_t getStepCount() const
        {
            return step_count_;
        }

        virtual void sleep_for(TTimeDelta dt)
        {
            if (dt <= 0)
                return;

            static constexpr std::chrono::duration<double> MinSleepDuration(0);
            TTimePoint start = nowNanos();
            //spin wait
            while (elapsedSince(start) < dt)
                std::this_thread::sleep_for(MinSleepDuration);
        }

        double getTrueScaleWrtWallClock()
        {
            TTimePoint wall_clock_now = Utils::getTimeSinceEpochNanos();
            TTimeDelta wall_clock_elapsed = elapsedBetween(wall_clock_now, wall_clock_start_);

            TTimePoint clock_now = nowNanos();
            TTimeDelta clock_elapsed = elapsedBetween(clock_now, getStart());

            return static_cast<double>(clock_elapsed) / wall_clock_elapsed;
        }

    private:
        template <typename T>
        using duration = std::chrono::duration<T>;

        uint64_t step_count_ = 0;
        TTimePoint wall_clock_start_;
    };
}
} //namespace
#endif

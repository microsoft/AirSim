// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_SteppableClock_hpp
#define airsim_core_SteppableClock_hpp

#include "ClockBase.hpp"
#include "Common.hpp"
#include <atomic>

namespace msr
{
namespace airlib
{

    class SteppableClock : public ClockBase
    {
    public:
        static constexpr real_T DefaultStepSize = 20E-3f;

        //Debug clock allows to advance the clock manually in arbitrary way
        //TTimePoint is nano seconds passed since some fixed point in time
        //step is how much we would advance the clock by default when calling step()
        //by default we advance by 20ms
        SteppableClock(TTimeDelta step = DefaultStepSize, TTimePoint start = 0)
            : current_(start), step_(step)
        {
            start_ = current_ = start ? start : Utils::getTimeSinceEpochNanos();
        }

        virtual ~SteppableClock() {}

        TTimePoint stepBy(TTimeDelta amount)
        {
            current_ = addTo(current_, amount);
            return current_;
        }

        virtual TTimePoint step() override
        {
            ClockBase::step();

            current_ = addTo(current_, step_);
            return current_;
        }

        TTimeDelta getStepSize() const
        {
            return step_;
        }

        virtual TTimePoint nowNanos() const override
        {
            return current_;
        }

        virtual TTimePoint getStart() const override
        {
            return start_;
        }

    private:
        std::atomic<TTimePoint> current_;
        std::atomic<TTimePoint> start_;

        TTimeDelta step_;
    };
}
} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_FrequencyLimiter_hpp
#define msr_airlib_FrequencyLimiter_hpp

#include "common/Common.hpp"
#include "UpdatableObject.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

class FrequencyLimiter : UpdatableObject {
public:
    FrequencyLimiter(real_T frequency = Utils::max<float>(), real_T startup_delay = 0)
    {
        initialize(frequency);
    }

    void initialize(real_T frequency, real_T startup_delay = 0)
    {
        if (Utils::isApproximatelyZero(frequency))
            interval_size_sec_ = 1E10;  //some high number
        else
            interval_size_sec_ = 1.0f / frequency;
        elapsed_total_sec_ = 0;
        elapsed_interval_sec_ = 0;
        last_elapsed_interval_sec_ = 0;
        start_timestamp_ms_ = Utils::getTimeSinceEpochMillis();
        update_count_ = 0;
        frequency_ = frequency;
        wait_complete_ = false;
        startup_delay_ = startup_delay;
        startup_complete_ = false;
        //reset call is not required as we do same in reset()
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        initialize(frequency_);
    }

    virtual void update(real_T dt) override
    {
        elapsed_total_sec_ += dt;
        elapsed_interval_sec_ += dt;
        ++update_count_;

        if (!startup_complete_) {
            if (Utils::isDefinitelyGreaterThan(startup_delay_, 0.0f))
                wait_complete_ = elapsed_interval_sec_ >= startup_delay_;
            else
                startup_complete_ = true;
        }
        
        if (startup_complete_)
            wait_complete_ = elapsed_interval_sec_ >= interval_size_sec_;
        
        if (wait_complete_) {
            last_elapsed_interval_sec_ = elapsed_interval_sec_;
            elapsed_interval_sec_ = 0;
            startup_complete_ = true;
        }
    }
    //*** End: UpdatableState implementation ***//


    ulong getTimestamp() const
    {
        return start_timestamp_ms_ + static_cast<ulong>(elapsed_total_sec_ * 1000);
    }

    real_T getElapsedTotalSec() const
    {
        return elapsed_total_sec_;
    }

    real_T getElapsedIntervalSec() const
    {
        return elapsed_interval_sec_;
    }

    real_T getLastElapsedIntervalSec() const
    {
        return last_elapsed_interval_sec_;
    }

    bool isWaitComplete() const
    {
        return wait_complete_;
    }

    bool isStartupComplete() const
    {
        return startup_complete_;
    }

    uint getUpdateCount() const
    {
        return update_count_;
    }

private:
    real_T interval_size_sec_;
    real_T elapsed_total_sec_;
    real_T elapsed_interval_sec_;
    real_T last_elapsed_interval_sec_;
    ulong start_timestamp_ms_;
    uint update_count_;
    real_T frequency_;
    real_T startup_delay_;
    bool wait_complete_;
    bool startup_complete_;
};

}} //namespace
#endif 

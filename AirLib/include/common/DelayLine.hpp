// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_DelayLine_hpp
#define common_utils_DelayLine_hpp

#include "common/Common.hpp"
#include "UpdatableObject.hpp"
#include <list>

namespace msr { namespace airlib {

template<typename T>
class DelayLine : UpdatableObject {
public:
    DelayLine()
    {}
    DelayLine(double delay) //in seconds
    {
        initialize(delay);
    }
    void initialize(double delay)  //in seconds
    {
        setDelay(delay);
        DelayLine::reset();
    }
    void setDelay(double delay)
    {
        delay_ = delay;
    }
    double getDelay() const
    {
        return delay_;
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        values_.clear();
        times_.clear();
        time_now = 0;
        last_time_ = -1;
        last_value_ = T();
    }

    virtual void update(real_T dt) override
    {
        time_now += dt;

        if (!times_.empty() && 
            time_now - times_.front() >= delay_) {

            last_value_ = values_.front();
            last_time_ = times_.front();

            times_.pop_front();
            values_.pop_front();
        }
    }
    //*** End: UpdatableState implementation ***//


    T getOutput() const
    {
        return last_value_;
    }
    double getOutputTime() const
    {
        return last_time_;
    }

    void push_back(const T& val, double time_offset = 0)
    {
        values_.push_back(val);
        times_.push_back(time_now + time_offset);
    }

private:
    template<typename TItem>
    using list = std::list<TItem>;

    list<T> values_;
    list<double> times_;
    double delay_;

    T last_value_;
    double last_time_ = -1;
    double time_now = 0;
};

}} //namespace
#endif

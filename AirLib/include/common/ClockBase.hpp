// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ClockBase_hpp
#define airsim_core_ClockBase_hpp

#include "Common.hpp"

namespace msr { namespace airlib {

class ClockBase {
public:
    virtual TTimePoint nowNanos() = 0;

    double elapsedSince(TTimePoint since)
    {
        return elapsedBetween(nowNanos(), since);
    }
    static double elapsedBetween(TTimePoint second, TTimePoint first)
    {
        return (second - first) / 1E9;
    }
    double updateSince(TTimePoint& since)
    {
        TTimePoint cur = nowNanos();
        double elapsed = elapsedBetween(cur, since);
        since = cur;
        return elapsed;
    }
};

}} //namespace
#endif

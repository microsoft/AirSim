// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ClockFactory_hpp
#define airsim_core_ClockFactory_hpp

#include "SimClock.hpp"

namespace msr { namespace airlib {

class ClockFactory {
public:
    static ClockBase* get()
    {
        static SimClock clock;
        
        return &clock;
    }

    //don't allow multiple instances of this class
    ClockFactory(ClockFactory const&) = delete;
    void operator=(ClockFactory const&) = delete;

private:
    //disallow instance creation
    ClockFactory(){}
};

}} //namespace
#endif

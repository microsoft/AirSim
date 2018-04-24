// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_SimModeApiBase_hpp
#define air_SimModeApiBase_hpp

#include "common/CommonStructs.hpp"
#include "VehicleApiBase.hpp"

namespace msr { namespace airlib {


class SimModeApiBase {
public:
    virtual VehicleApiBase* getVehicleApi() = 0;
    virtual ~SimModeApiBase() = default;
};


}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleCamera_hpp
#define air_VehicleCamera_hpp

#include "DroneControllerBase.hpp"

namespace msr {
    namespace airlib {

        // This is an abstraction for cameras associated with a vehicle.  Each camera has a unique id.
        class VehicleCamera
        {
        public:
            virtual int getId() = 0;
            virtual bool getScreenShot(DroneControllerBase::ImageType imageType, std::vector<uint8_t>& compressedPng) = 0;
        };
    }
};

#endif
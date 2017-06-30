// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleCamera_hpp
#define air_VehicleCamera_hpp

#include <vector>
#include "common/common_utils/EnumFlags.hpp"

namespace msr { namespace airlib {
        // This is an abstraction for cameras associated with a vehicle.  Each camera has a unique id.
        class VehicleCamera
        {
        public:
            enum class ImageType : uint {
                None = 0,
                Scene = 1, 
                Depth = 2, 
                Segmentation = 4,
                All = 255
            };
            typedef common_utils::EnumFlags<ImageType>  ImageTypeFlags;

        public:
            virtual int getId() = 0;
            virtual bool getScreenShot(ImageType imageType, std::vector<uint8_t>& compressedPng) = 0;
        };
    }
};

#endif
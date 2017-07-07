// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleCameraBase_hpp
#define air_VehicleCameraBase_hpp

#include "common/Common.hpp"
#include "common/common_utils/EnumFlags.hpp"

namespace msr { namespace airlib {
        // This is an abstraction for cameras associated with a vehicle.  Each camera has a unique id.
        class VehicleCameraBase
        {
        public: //types
            enum class ImageType_ : uint {
                None = 0,
                Scene = 1, 
                Depth = 2, 
                Segmentation = 4,
                All = 255
            };
            typedef common_utils::EnumFlags<ImageType_>  ImageType;

            struct ImageResponse {
                vector<uint8_t> image_data;
                Vector3r camera_position = Vector3r::Zero();
                Quaternionr camera_orientation = Quaternionr::Identity();
                TTimePoint time_stamp = 0;
                std::string message;
            };

        public: //methods
            virtual ImageResponse getImage(ImageType image_type) = 0;
        };


}} //namespace
#endif
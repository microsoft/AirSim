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
            enum class ImageType : uint { //this indexes to array
                Scene = 0, 
                DepthMeters, 
                DepthVis, 
                DisparityNormalized,
                Segmentation,
                Normals,
                Count //must be last
            };

            struct ImageResponse {
                vector<uint8_t> image_data_uint8;
                vector<float> image_data_float;

                Vector3r camera_position = Vector3r::Zero();
                Quaternionr camera_orientation = Quaternionr::Identity();
                TTimePoint time_stamp = 0;
                std::string message;
                bool pixels_as_float = false;
                bool compress = true;
                int width = 0, height = 0;
            };

        public: //methods
            virtual ImageResponse getImage(ImageType image_type, bool pixels_as_float, bool compress) = 0;
        };


}} //namespace
#endif
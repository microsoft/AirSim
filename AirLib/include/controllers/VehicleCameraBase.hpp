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
                DepthPlanner, 
                DepthPerspective,
                DepthVis, 
                DisparityNormalized,
                Segmentation,
                SurfaceNormals,
                Count //must be last
            };

            struct ImageRequest {
                uint8_t camera_id;
                VehicleCameraBase::ImageType image_type;
                bool pixels_as_float;
                bool compress;

                ImageRequest()
                {}

                ImageRequest(uint8_t camera_id_val, VehicleCameraBase::ImageType image_type_val, bool pixels_as_float_val = false, bool compress_val = true)
                {
                    camera_id = camera_id_val;
                    image_type = image_type_val;
                    pixels_as_float = pixels_as_float_val;
                    compress = compress_val;
                }
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
                ImageType image_type;
            };

        public: //methods
            virtual ImageResponse getImage(ImageType image_type, bool pixels_as_float, bool compress) = 0;
        };


}} //namespace
#endif
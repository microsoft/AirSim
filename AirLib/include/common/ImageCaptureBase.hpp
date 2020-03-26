// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ImageCaptureBase_hpp
#define air_ImageCaptureBase_hpp

#pragma once
#include "common/Common.hpp"
#include "common/common_utils/EnumFlags.hpp"
#include <functional>

namespace msr { namespace airlib {

// This is an abstraction for cameras associated with a vehicle.  Each camera has a unique id.
class ImageCaptureBase
{
public: //types
    enum class ImageType : int { //this indexes to array, -1 is special to indicate main camera
        Scene = 0, 
        DepthPlanner, 
        DepthPerspective,
        DepthVis, 
        DisparityNormalized,
        Segmentation,
        SurfaceNormals,
        Infrared,
        Count //must be last
    };

    struct ImageRequest {
        std::string camera_name;
        ImageCaptureBase::ImageType image_type = ImageCaptureBase::ImageType::Scene;
        bool pixels_as_float = false;
        bool compress = true;

        ImageRequest()
        {}

        ImageRequest(const std::string& camera_name_val, ImageCaptureBase::ImageType image_type_val, bool pixels_as_float_val = false, bool compress_val = true)
        {
            camera_name = camera_name_val;
            image_type = image_type_val;
            pixels_as_float = pixels_as_float_val;
            compress = compress_val;
        }
    };

    struct ImageResponse {
        std::unique_ptr<vector<uint8_t>, std::function<void(vector<uint8_t>*)>> image_data_uint8 = nullptr;
        vector<float> image_data_float;

        std::string camera_name;
        Vector3r camera_position = Vector3r::Zero();
        Quaternionr camera_orientation = Quaternionr::Identity();
        TTimePoint time_stamp = 0;
        std::string message;
        bool pixels_as_float = false;
        bool compress = true;
        int width = 0, height = 0;
        ImageType image_type;

        ImageResponse() : image_data_uint8(nullptr), camera_name(""), camera_position(Vector3r::Zero()), camera_orientation(Quaternionr::Identity()), time_stamp(0), message(""), pixels_as_float(false), compress(true), width(0), height(0), image_type(ImageType::Scene) {}

        ImageResponse(const ImageResponse& other)
        {
            image_data_uint8 = std::unique_ptr<vector<uint8_t>, std::function<void(vector<uint8_t>*)>>(other.image_data_uint8.get(), std::bind(&ImageResponse::CopyDeleter, this, std::placeholders::_1));
            camera_name = other.camera_name;
            camera_position = other.camera_position;
            camera_orientation = other.camera_orientation;
            time_stamp = other.time_stamp;
            message = other.message;
            pixels_as_float = other.pixels_as_float;
            compress = other.compress;
            width = other.width;
            height = other.height;
            image_type = other.image_type;
        }

        ImageResponse& operator=(const ImageResponse& other)
        {
            image_data_uint8 = std::unique_ptr<vector<uint8_t>, std::function<void(vector<uint8_t>*)>>(other.image_data_uint8.get(), std::bind(&ImageResponse::CopyDeleter, this, std::placeholders::_1));
            camera_name = other.camera_name;
            camera_position = other.camera_position;
            camera_orientation = other.camera_orientation;
            time_stamp = other.time_stamp;
            message = other.message;
            pixels_as_float = other.pixels_as_float;
            compress = other.compress;
            width = other.width;
            height = other.height;
            image_type = other.image_type;
            return *this;
        }

    private:
        void CopyDeleter(vector<uint8_t>* buf) {
            buf = nullptr; //Avoid error for unreferenced formal parameter.
        } //Copies of an ImageResponse effectively contain weak pointers. The original response handles deleting the buffer. Ultimately, response copying should be removed from everywhere.
    };

public: //methods
    virtual void getImages(const std::vector<ImageRequest>& requests, std::vector<ImageResponse>& responses) const = 0;
    virtual void getImage(const ImageRequest& request, ImageResponse& response) const = 0;
};


}} //namespace
#endif
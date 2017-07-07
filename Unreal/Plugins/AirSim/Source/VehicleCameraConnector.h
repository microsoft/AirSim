#pragma once

#include "controllers/VehicleCameraBase.hpp"
#include "PIPCamera.h"


class VehicleCameraConnector : public VehicleCameraBase
{
public:
    typedef msr::airlib::VehicleCameraBase::ImageType ImageType;
    typedef msr::airlib::VehicleCameraBase::ImageType_ ImageType_;

    VehicleCameraConnector(APIPCamera* camera);
    virtual ImageResponse getImage(ImageType image_type, bool pixels_as_float, bool compress) override;

private:
    msr::airlib::VehicleCameraBase::ImageResponse getSceneCaptureImage(ImageType image_type, bool pixels_as_float, bool compress);

    void addScreenCaptureHandler(UWorld *world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

private:
    APIPCamera* camera_;
    std::vector<uint8_t> last_compressed_png_;
};
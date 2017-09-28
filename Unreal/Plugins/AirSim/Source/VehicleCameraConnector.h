#pragma once

#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "controllers/VehicleCameraBase.hpp"


class VehicleCameraConnector : public msr::airlib::VehicleCameraBase
{
public:
    typedef msr::airlib::VehicleCameraBase::ImageType ImageType;

    VehicleCameraConnector(APIPCamera* camera);
    virtual ImageResponse getImage(ImageType image_type, bool pixels_as_float, bool compress) override;

private:
    msr::airlib::VehicleCameraBase::ImageResponse getSceneCaptureImage(ImageType image_type, 
        bool pixels_as_float, bool compress, bool use_safe_method);

    void addScreenCaptureHandler(UWorld *world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

private:
    APIPCamera* camera_;
    std::vector<uint8_t> last_compressed_png_;
};
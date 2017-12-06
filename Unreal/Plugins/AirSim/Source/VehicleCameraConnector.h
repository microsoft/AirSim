#pragma once

#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "controllers/ImageCaptureBase.hpp"


class VehicleCameraConnector : public msr::airlib::ImageCaptureBase
{
public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    VehicleCameraConnector(APIPCamera* camera);
    virtual ~VehicleCameraConnector();
    virtual ImageResponse getImage(ImageType image_type, bool pixels_as_float, bool compress) override;

private:
    msr::airlib::ImageCaptureBase::ImageResponse getSceneCaptureImage(ImageType image_type, 
        bool pixels_as_float, bool compress, bool use_safe_method);

    void addScreenCaptureHandler(UWorld *world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

private:
    APIPCamera* camera_;
    std::vector<uint8_t> last_compressed_png_;
};
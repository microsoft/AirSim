#pragma once

#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "common/ImageCaptureBase.hpp"


class UnrealImageCapture : public msr::airlib::ImageCaptureBase
{
public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    UnrealImageCapture(const std::vector<APIPCamera*>& cameras);
    virtual ~UnrealImageCapture();

    virtual void getImages(const std::vector<ImageRequest>& requests, std::vector<ImageResponse>& responses) override;

private:
    void getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, 
        std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method);

    void addScreenCaptureHandler(UWorld *world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

    void updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request);

private:
    std::vector<APIPCamera*> cameras_;
    std::vector<uint8_t> last_compressed_png_;
};
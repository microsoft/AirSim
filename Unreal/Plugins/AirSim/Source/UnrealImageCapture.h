#pragma once

#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "BufferPool.h"

class UnrealImageCapture : public msr::airlib::ImageCaptureBase
{
public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras);
    virtual ~UnrealImageCapture();

    virtual void getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const override;
    virtual void getImage(const ImageRequest& request, ImageResponse& response) const override;

private:
    BufferPool<uint8_t> *BufferPool_ = new BufferPool<uint8_t>();
    BufferPool<float> *BufferPool_float_ = new BufferPool<float>();

    void getSceneCaptureImage(const std::string& camera_name, ImageCaptureBase::ImageType image_type, ImageResponse &response) const;

    void addScreenCaptureHandler(UWorld *world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

private:
    const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras_;
    std::vector<uint8_t> last_compressed_png_;
};

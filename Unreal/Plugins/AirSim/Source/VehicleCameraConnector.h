#pragma once

#include "controllers/VehicleCamera.hpp"
#include "PIPCamera.h"


class VehicleCameraConnector : public VehicleCamera
{
public:
    VehicleCameraConnector(APIPCamera* camera, int id);
    int getId() override;
    bool getScreenShot(DroneControllerBase::ImageType imageType, std::vector<uint8_t>& compressedPng) override;


private:
    bool getScreenshotScreen(DroneControllerBase::ImageType imageType, std::vector<uint8_t>& compressedPng);
    bool getScreenshotSceneCapture(DroneControllerBase::ImageType imageType, std::vector<uint8_t>& compressedPng);
    void addScreenCaptureHandler(UWorld *world);

private:
    APIPCamera* camera;
    int id;
    std::vector<uint8_t> last_compressed_png_;
};
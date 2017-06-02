#include "AirSim.h"
#include "PIPCamera.h"
#include "controllers/VehicleCamera.hpp"
#include "RenderRequest.h"

class VehicleCameraConnector : public VehicleCamera
{
    APIPCamera* camera;
    int id;
public:
    VehicleCameraConnector(APIPCamera* camera, int id) {
        this->camera = camera;
        this->id = id;
    }

    int getId() override {
        return id;
    }

    bool getScreenShot(DroneControllerBase::ImageType imageType, std::vector<uint8_t>& compressedPng) override 
    {

        if (this->camera == nullptr) {
            return false;
        }
        if (imageType == DroneControllerBase::ImageType::None) {
            return false;
        }
        EPIPCameraType pip_type;
        //TODO: merge these two different types?
        switch (imageType) {
        case DroneControllerBase::ImageType::Scene:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SCENE; break;
        case DroneControllerBase::ImageType::Depth:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_DEPTH; break;
        case DroneControllerBase::ImageType::Segmentation:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SEG; break;
        default:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_NONE;
        }

        using namespace msr::airlib;
        USceneCaptureComponent2D* capture = camera->getCaptureComponent(pip_type, true);
        if (capture == nullptr) {
            UAirBlueprintLib::LogMessage(TEXT("Can't take screenshot because eithercamera type is not active"), TEXT(""), LogDebugLevel::Failure);
            return false;
        }

        if (capture->TextureTarget == nullptr) {
            UAirBlueprintLib::LogMessage(TEXT("Can't take screenshot because texture target is null"), TEXT(""), LogDebugLevel::Failure);
            return false;
        }

        UTextureRenderTarget2D* textureTarget = capture->TextureTarget;

        TArray<uint8> image;
        RenderRequest request;
        request.getScreenshot(textureTarget, image);

        // copy data across to std::vector.
        compressedPng = std::vector<uint8_t>(image.GetData(), image.GetData() + image.Num());
        return true;
    }
};
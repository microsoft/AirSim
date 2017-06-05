#include "AirSim.h"
#include "PIPCamera.h"
#include "controllers/VehicleCamera.hpp"
#include "RenderRequest.h"
#include "SimHUD/SimHUD.h"
#include <chrono>

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
        ASimHUD* hud = ASimHUD::GetInstance();
        EPIPCameraType pip_type;
        bool visibilityChanged = false;
        //TODO: merge these two different types?
        switch (imageType) {
        case DroneControllerBase::ImageType::Scene:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SCENE; 
            if (!hud->isPIPSceneVisible()) {
                hud->inputEventTogglePIPScene();
                visibilityChanged = true;
            }
            break;
        case DroneControllerBase::ImageType::Depth:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_DEPTH;
            if (!hud->isPIPDepthVisible()) {
                hud->inputEventTogglePIPDepth();
                visibilityChanged = true;
            }
            break;
        case DroneControllerBase::ImageType::Segmentation:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SEG;
            if (!hud->isPIPSegVisible()) {
                hud->inputEventTogglePIPSeg();
                visibilityChanged = true;
            }
            break;
        default:
            pip_type = EPIPCameraType::PIP_CAMERA_TYPE_NONE;
        }

        if (visibilityChanged) {
            // Wait for render so that view is ready for capture
            std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
            // not sure why this doesn't work.
            //DECLARE_CYCLE_STAT(TEXT("FNullGraphTask.CheckRenderStatus"), STAT_FNullGraphTask_CheckRenderStatus, STATGROUP_TaskGraphTasks);
            //auto renderStatus = TGraphTask<FNullGraphTask>::CreateTask(NULL).ConstructAndDispatchWhenReady(GET_STATID(STAT_FNullGraphTask_CheckRenderStatus), ENamedThreads::RenderThread);
            //FTaskGraphInterface::Get().WaitUntilTaskCompletes(renderStatus);
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
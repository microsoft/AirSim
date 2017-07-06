#include "AirSim.h"
#include "PIPCamera.h"
#include "AirBlueprintLib.h"
#include "ImageUtils.h"
#include <string>

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    camera_ = UAirBlueprintLib::GetActorComponent<UCameraComponent>(this, TEXT("CameraComponent"));

    screen_capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    depth_capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthCaptureComponent"));
    seg_capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();

    scene_render_target_ = NewObject<UTextureRenderTarget2D>();
    scene_render_target_->InitAutoFormat(256, 144);
    scene_render_target_->bHDR = false;
    scene_render_target_->TargetGamma = 1.05f; // GEngine->GetDisplayGamma();

    depth_render_target_ = NewObject<UTextureRenderTarget2D>();
    depth_render_target_->InitAutoFormat(256, 144);
    seg_render_target_ = NewObject<UTextureRenderTarget2D>();
    seg_render_target_->InitAutoFormat(256, 144);
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    scene_render_target_ = nullptr;
    depth_render_target_ = nullptr;
    seg_render_target_ = nullptr;
}

void APIPCamera::showToScreen()
{
    camera_->Activate();
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    controller->SetViewTarget(this);
}

void APIPCamera::disableAll()
{
    disableMain();
    disableAllPIP();
}


EPIPCameraType APIPCamera::toggleEnableCameraTypes(EPIPCameraType types)
{
    setEnableCameraTypes(getEnableCameraTypes() ^ types);
    return getEnableCameraTypes() & types;
}

EPIPCameraType APIPCamera::getEnableCameraTypes()
{
    return enabled_camera_types_;
}

void APIPCamera::setEnableCameraTypes(EPIPCameraType types)
{
    enableCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_DEPTH, (types & EPIPCameraType::PIP_CAMERA_TYPE_DEPTH) != EPIPCameraType::PIP_CAMERA_TYPE_NONE);
    enableCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_SCENE, (types & EPIPCameraType::PIP_CAMERA_TYPE_SCENE) != EPIPCameraType::PIP_CAMERA_TYPE_NONE);
    enableCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_SEG, (types & EPIPCameraType::PIP_CAMERA_TYPE_SEG) != EPIPCameraType::PIP_CAMERA_TYPE_NONE);
    enabled_camera_types_ = types;
}

void APIPCamera::enableCaptureComponent(const EPIPCameraType type, bool is_enabled)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false);
    if (capture != nullptr) {
        if (is_enabled) {
            capture->TextureTarget = getRenderTarget(type, false);
            capture->Activate();
        }
        else {
            capture->Deactivate();
            capture->TextureTarget = nullptr;
        }
    }
    //else nothing to enable
}

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const EPIPCameraType type, bool if_active)
{
    switch (type) {
    case EPIPCameraType::PIP_CAMERA_TYPE_SCENE:
        if (!if_active || (static_cast<uint8>(enabled_camera_types_) & static_cast<uint8>(EPIPCameraType::PIP_CAMERA_TYPE_SCENE)))
            return scene_render_target_;
        return nullptr;
    case EPIPCameraType::PIP_CAMERA_TYPE_DEPTH:
        if (!if_active || (static_cast<uint8>(enabled_camera_types_) & static_cast<uint8>(EPIPCameraType::PIP_CAMERA_TYPE_DEPTH)))
            return depth_render_target_;
        return nullptr;
    case EPIPCameraType::PIP_CAMERA_TYPE_SEG:
        if (!if_active || (static_cast<uint8>(enabled_camera_types_) & static_cast<uint8>(EPIPCameraType::PIP_CAMERA_TYPE_SEG)))
            return seg_render_target_;
        return nullptr;
    case EPIPCameraType::PIP_CAMERA_TYPE_NONE:
        return nullptr;
    default:
        return nullptr;
        UAirBlueprintLib::LogMessageString("Cannot get render target because camera type is not recognized", std::to_string(static_cast<uint8>(type)), LogDebugLevel::Failure, 60);
    }

}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const EPIPCameraType type, bool if_active)
{
    switch (type) {
    case EPIPCameraType::PIP_CAMERA_TYPE_SCENE:
        if (!if_active || (static_cast<uint8>(enabled_camera_types_) & static_cast<uint8>(EPIPCameraType::PIP_CAMERA_TYPE_SCENE)))
            return screen_capture_;
        return nullptr;
    case EPIPCameraType::PIP_CAMERA_TYPE_DEPTH:
        if (!if_active || (static_cast<uint8>(enabled_camera_types_) & static_cast<uint8>(EPIPCameraType::PIP_CAMERA_TYPE_DEPTH)))
            return depth_capture_;
        return nullptr;
    case EPIPCameraType::PIP_CAMERA_TYPE_SEG:
        if (!if_active || (static_cast<uint8>(enabled_camera_types_) & static_cast<uint8>(EPIPCameraType::PIP_CAMERA_TYPE_SEG)))
            return seg_capture_;
        return nullptr;
    case EPIPCameraType::PIP_CAMERA_TYPE_NONE:
        return nullptr;
    default:
        return nullptr;
        UAirBlueprintLib::LogMessageString("Cannot get capture component because camera type is not recognized", std::to_string(static_cast<uint8>(type)), LogDebugLevel::Failure, 60);
    }

}


void APIPCamera::disableAllPIP()
{
    enableCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_SCENE, false);
    enableCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_DEPTH, false);
    enableCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_SEG, false);
}


void APIPCamera::disableMain()
{
    camera_->Deactivate();
}





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

    screen_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    depth_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    seg_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    
    scene_render_target_ = NewObject<UTextureRenderTarget2D>();
    scene_render_target_->InitAutoFormat(960, 540); //256 X 144, X 480
    //scene_render_target_->bHDR = false;
    scene_render_target_->TargetGamma = 1.0f; // GEngine->GetDisplayGamma();

    depth_render_target_ = NewObject<UTextureRenderTarget2D>();
    depth_render_target_->InitAutoFormat(960, 540);
    depth_render_target_->TargetGamma = 1.0f;

    seg_render_target_ = NewObject<UTextureRenderTarget2D>();
    seg_render_target_->InitAutoFormat(960, 540);
    seg_render_target_->TargetGamma = 1.0f;
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


APIPCamera::ImageType APIPCamera::toggleEnableCameraTypes(APIPCamera::ImageType types)
{
    setEnableCameraTypes(types ^ getEnableCameraTypes());
    return types ^ getEnableCameraTypes();
}

APIPCamera::ImageType APIPCamera::getEnableCameraTypes()
{
    return enabled_camera_types_;
}

void APIPCamera::setEnableCameraTypes(APIPCamera::ImageType types)
{
    enableCaptureComponent(ImageType_::Depth, (types & ImageType_::Depth));
    enableCaptureComponent(ImageType_::Scene, (types & ImageType_::Scene));
    enableCaptureComponent(ImageType_::Segmentation, (types & ImageType_::Segmentation));
    enabled_camera_types_ = types;
}

void APIPCamera::enableCaptureComponent(const APIPCamera::ImageType type, bool is_enabled)
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

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const APIPCamera::ImageType type, bool if_active)
{
    switch (type.toEnum()) {
    case ImageType_::Scene:
        if (!if_active || (enabled_camera_types_ & ImageType_::Scene))
            return scene_render_target_;
        return nullptr;
    case ImageType_::Depth:
        if (!if_active || (enabled_camera_types_ & ImageType_::Depth))
            return depth_render_target_;
        return nullptr;
    case ImageType_::Segmentation:
        if (!if_active || (enabled_camera_types_ & ImageType_::Segmentation))
            return seg_render_target_;
        return nullptr;
    case ImageType_::None:
        return nullptr;
    default:
        return nullptr;
        UAirBlueprintLib::LogMessageString("Cannot get render target because camera type is not recognized", std::to_string(type), LogDebugLevel::Failure);
    }

}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const APIPCamera::ImageType type, bool if_active)
{
    switch (type.toEnum()) {
    case ImageType_::Scene:
        if (!if_active || (enabled_camera_types_ & ImageType_::Scene))
            return screen_capture_;
        return nullptr;
    case ImageType_::Depth:
        if (!if_active || (enabled_camera_types_ & ImageType_::Depth))
            return depth_capture_;
        return nullptr;
    case ImageType_::Segmentation:
        if (!if_active || (enabled_camera_types_ & ImageType_::Segmentation))
            return seg_capture_;
        return nullptr;
    case ImageType_::None:
        return nullptr;
    default:
        return nullptr;
        UAirBlueprintLib::LogMessageString("Cannot get capture component because camera type is not recognized", std::to_string(static_cast<uint8>(type.toEnum())), LogDebugLevel::Failure, 60);
    }

}


void APIPCamera::disableAllPIP()
{
    enableCaptureComponent(ImageType_::Scene, false);
    enableCaptureComponent(ImageType_::Depth, false);
    enableCaptureComponent(ImageType_::Segmentation, false);
}


void APIPCamera::disableMain()
{
    camera_->Deactivate();
}





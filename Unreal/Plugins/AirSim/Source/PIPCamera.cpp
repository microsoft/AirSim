#include "AirSim.h"
#include <string>
#include <exception>
#include "PIPCamera.h"
#include "AirBlueprintLib.h"
#include "ImageUtils.h"

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    camera_ = UAirBlueprintLib::GetActorComponent<UCameraComponent>(this, TEXT("CameraComponent"));

    screen_capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    depth_capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthCaptureComponent"));
    seg_capture_ = UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));

    //set default for brigher images
    scene_capture_settings_.target_gamma = CaptureSettings::kSceneTargetGamma;
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();

    screen_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    depth_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    seg_capture_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    
    scene_render_target_ = NewObject<UTextureRenderTarget2D>();
    setCaptureSettings(ImageType_::Scene, scene_capture_settings_);
    //scene_render_target_->bHDR = false;

    depth_render_target_ = NewObject<UTextureRenderTarget2D>();
    setCaptureSettings(ImageType_::Depth, depth_capture_settings_);

    seg_render_target_ = NewObject<UTextureRenderTarget2D>();
    setCaptureSettings(ImageType_::Segmentation, seg_capture_settings_);
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    scene_render_target_ = nullptr;
    depth_render_target_ = nullptr;
    seg_render_target_ = nullptr;
}

void APIPCamera::showToScreen()
{
    camera_->SetVisibility(true);
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
}

APIPCamera::CaptureSettings APIPCamera::getCaptureSettings(ImageType_ type)
{
    switch (type)
    {
    case ImageType_::Scene: return scene_capture_settings_;
    case ImageType_::Depth: return depth_capture_settings_;
    case ImageType_::Segmentation: return seg_capture_settings_;
    default:
        throw std::invalid_argument("the ImageType specified for getCaptureSettings is not recognized");
    }
}

void APIPCamera::setCaptureSettings(APIPCamera::ImageType_ type, const APIPCamera::CaptureSettings& settings)
{
    switch (type)
    {
    case ImageType_::Scene: {
        scene_capture_settings_ = settings; 
        updateCaptureComponentSettings(screen_capture_, scene_render_target_, scene_capture_settings_);
        break;
    }
    case ImageType_::Depth: {
        depth_capture_settings_ = settings;
        updateCaptureComponentSettings(depth_capture_, depth_render_target_, depth_capture_settings_);
        break;
    }
    case ImageType_::Segmentation: {
        seg_capture_settings_ = settings;
        updateCaptureComponentSettings(seg_capture_, seg_render_target_, seg_capture_settings_);
        break;
    }
    default:
        throw std::invalid_argument("the ImageType specified for setCaptureSettings is not recognized");
    }
}

void APIPCamera::updateCaptureComponentSettings(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, const CaptureSettings& settings)
{
    if (render_target) {
        render_target->InitAutoFormat(settings.width, settings.height); //256 X 144, X 480
        render_target->TargetGamma = settings.target_gamma;
    }
    //else we will set this after this components get created

    if (capture) {
        if (!std::isnan(settings.fov_degrees))
            capture->FOVAngle = settings.fov_degrees;
        if (!std::isnan(settings.motion_blur_amount))
            capture->PostProcessSettings.MotionBlurAmount = settings.motion_blur_amount;

        capture->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;        
        if (!std::isnan(settings.auto_exposure_speed))
            capture->PostProcessSettings.AutoExposureSpeedDown = capture->PostProcessSettings.AutoExposureSpeedUp = settings.auto_exposure_speed;
        if (!std::isnan(settings.auto_exposure_max_brightness))
            capture->PostProcessSettings.AutoExposureMaxBrightness = settings.auto_exposure_max_brightness;
        if (!std::isnan(settings.auto_exposure_min_brightness))
            capture->PostProcessSettings.AutoExposureMinBrightness = settings.auto_exposure_min_brightness;
        if (!std::isnan(settings.auto_exposure_bias))
            capture->PostProcessSettings.AutoExposureBias = settings.auto_exposure_bias;
        if (!std::isnan(settings.auto_exposure_low_percent))
            capture->PostProcessSettings.AutoExposureLowPercent = settings.auto_exposure_low_percent;        
        if (!std::isnan(settings.auto_exposure_high_percent))
            capture->PostProcessSettings.AutoExposureHighPercent = settings.auto_exposure_high_percent;    
        if (!std::isnan(settings.auto_exposure_histogram_log_min))
            capture->PostProcessSettings.HistogramLogMin = settings.auto_exposure_histogram_log_min;    
        if (!std::isnan(settings.auto_exposure_histogram_log_max))
            capture->PostProcessSettings.HistogramLogMax = settings.auto_exposure_histogram_log_max;    

    }
    //else we will set this after this components get created
}

void APIPCamera::enableCaptureComponent(const APIPCamera::ImageType type, bool is_enabled)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false);
    if (capture != nullptr) {
        if (is_enabled) {
            //do not make unnecessory calls to Activate() which otherwise causes crash in Unreal
            if (!capture->IsActive() || capture->TextureTarget == nullptr) {
                capture->TextureTarget = getRenderTarget(type, false);
                capture->Activate();
            }
            enabled_camera_types_ |= type;
        }
        else {
            if (capture->IsActive() || capture->TextureTarget != nullptr) {
                capture->Deactivate();
                capture->TextureTarget = nullptr;
            }
            enabled_camera_types_ &= ~type;
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
        UAirBlueprintLib::LogMessageString("Cannot get render target because camera type is not recognized", std::to_string(static_cast<uint8>(type.toEnum())), LogDebugLevel::Failure);
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
        UAirBlueprintLib::LogMessageString("Cannot get capture component because camera type is not recognized", std::to_string(static_cast<uint8>(type.toEnum())), LogDebugLevel::Failure);
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
    camera_->SetVisibility(false);
}





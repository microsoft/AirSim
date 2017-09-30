#include "PIPCamera.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include <string>
#include <exception>
#include "AirBlueprintLib.h"
#include "ImageUtils.h"

unsigned int APIPCamera::imageTypeCount()
{
    return Utils::toNumeric(ImageType::Count);
}

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    camera_ = UAirBlueprintLib::GetActorComponent<UCameraComponent>(this, TEXT("CameraComponent"));
    captures_.Init(nullptr, imageTypeCount());
    render_targets_.Init(nullptr, imageTypeCount());

    captures_[Utils::toNumeric(ImageType::Scene)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPlanner)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPlannerCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPerspective)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPerspectiveCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthVis)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DisparityNormalized)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DisparityNormalizedCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Segmentation)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::SurfaceNormals)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("NormalsCaptureComponent"));
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();
    
    //set default for brigher images 
    capture_settings_.assign(imageTypeCount(), CaptureSettings());
    capture_settings_[Utils::toNumeric(ImageType::Scene)].target_gamma = CaptureSettings::kSceneTargetGamma;

    //by default all image types are disabled
    camera_type_enabled_.assign(imageTypeCount(), false);

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

        render_targets_[image_type] = NewObject<UTextureRenderTarget2D>();
        updateCaptureComponentSettings(captures_[image_type], render_targets_[image_type], capture_settings_[image_type]);
    }
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        captures_[image_type] = nullptr;
        render_targets_[image_type] = nullptr;
    }
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

bool APIPCamera::getCameraTypeEnabled(ImageType type) const
{
    return camera_type_enabled_[Utils::toNumeric(type)];
}

void APIPCamera::setCameraTypeEnabled(ImageType type, bool enabled)
{
    enableCaptureComponent(type, enabled);
}

const APIPCamera::CaptureSettings& APIPCamera::getCaptureSettings(ImageType type)
{
    return capture_settings_[Utils::toNumeric(type)];
}

void APIPCamera::setCaptureSettings(APIPCamera::ImageType type, const APIPCamera::CaptureSettings& settings)
{
    unsigned int image_type = Utils::toNumeric(type);

    capture_settings_[image_type] = settings;
    updateCaptureComponentSettings(captures_[image_type], render_targets_[image_type], 
        capture_settings_[image_type]);
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
        }
        else {
            if (capture->IsActive() || capture->TextureTarget != nullptr) {
                capture->Deactivate();
                capture->TextureTarget = nullptr;
            }
        }
        camera_type_enabled_[Utils::toNumeric(type)] = is_enabled;
    }
    //else nothing to enable
}

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const APIPCamera::ImageType type, bool if_active)
{
    unsigned int image_type = Utils::toNumeric(type);

    if (!if_active || camera_type_enabled_[image_type])
        return render_targets_[image_type];
    return nullptr;
}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const APIPCamera::ImageType type, bool if_active)
{
    unsigned int image_type = Utils::toNumeric(type);

    if (!if_active || camera_type_enabled_[image_type])
        return captures_[image_type];
    return nullptr;
}

void APIPCamera::disableAllPIP()
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        enableCaptureComponent(Utils::toEnum<ImageType>(image_type), false);
    }
}


void APIPCamera::disableMain()
{
    camera_->Deactivate();
    camera_->SetVisibility(false);
}





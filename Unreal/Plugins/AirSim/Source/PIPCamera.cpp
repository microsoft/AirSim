#include "PIPCamera.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include <string>
#include <exception>
#include "Materials/MaterialInstanceDynamic.h"
#include "AirBlueprintLib.h"
#include "ImageUtils.h"


APIPCamera::APIPCamera()
{
    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraSensorNoise.CameraSensorNoise'"));
    if (mat_finder.Succeeded())
    {
        noise_material_static_ = mat_finder.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create noise material for the PIPCamera", 
            "", LogDebugLevel::Failure);
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
    captures_[Utils::toNumeric(ImageType::Infrared)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("InfraredCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::SurfaceNormals)] = 
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("NormalsCaptureComponent"));
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();
    
    //Computer transform using the actor that this camera is attached to
    //if this is free floting camera like external observer camera then just use 
    //itself to select origin point
    auto* p = this->GetAttachParentActor();
    ned_transform_.initialize(p ? p : this);

    noise_materials_.AddZeroed(imageTypeCount() + 1);

    //by default all image types are disabled
    camera_type_enabled_.assign(imageTypeCount(), false);

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

        render_targets_[image_type] = NewObject<UTextureRenderTarget2D>();
    }
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (noise_materials_.Num()) {
        for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
            if (noise_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(noise_materials_[image_type + 1]);
        }
        if (noise_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(noise_materials_[0]);
    }

    noise_material_static_ = nullptr;
    noise_materials_.Reset();

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        captures_[image_type] = nullptr;
        render_targets_[image_type] = nullptr;
    }
}


unsigned int APIPCamera::imageTypeCount()
{
    return Utils::toNumeric(ImageType::Count);
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

void APIPCamera::setImageTypeSettings(int image_type, const APIPCamera::CaptureSetting& capture_setting, 
    const APIPCamera::NoiseSetting& noise_setting)
{
    if (image_type >= 0) { //scene captue components
        updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type],
            capture_setting, ned_transform_);

        setNoiseMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings, noise_setting);
    }
    else { //camera component
        updateCameraSetting(camera_, capture_setting, ned_transform_);

        setNoiseMaterial(image_type, camera_, camera_->PostProcessSettings, noise_setting);
    }
}

void APIPCamera::updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, 
    const CaptureSetting& setting, const NedTransform& ned_transform)
{
    render_target->InitAutoFormat(setting.width, setting.height); //256 X 144, X 480
    if (!std::isnan(setting.target_gamma))
        render_target->TargetGamma = setting.target_gamma;

    capture->ProjectionType = static_cast<ECameraProjectionMode::Type>(setting.projection_mode);

    if (!std::isnan(setting.fov_degrees))
        capture->FOVAngle = setting.fov_degrees;
    if (capture->ProjectionType == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        capture->OrthoWidth = ned_transform.toNeuUU(setting.ortho_width);

    updateCameraPostProcessingSetting(capture->PostProcessSettings, setting);
}

void APIPCamera::updateCameraSetting(UCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform)
{
    //if (!std::isnan(setting.target_gamma))
    //    camera-> = setting.target_gamma;

    camera->SetProjectionMode(static_cast<ECameraProjectionMode::Type>(setting.projection_mode));

    if (!std::isnan(setting.fov_degrees))
        camera->SetFieldOfView(setting.fov_degrees);
    if (camera->ProjectionMode == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        camera->SetOrthoWidth(ned_transform.toNeuUU(setting.ortho_width));

    updateCameraPostProcessingSetting(camera->PostProcessSettings, setting);
}

msr::airlib::Pose APIPCamera::getPose() const
{
    return Pose(ned_transform_.toNedMeters(this->GetActorLocation()),
        ned_transform_.toQuaternionr(this->GetActorRotation().Quaternion(), true));
}

void APIPCamera::updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting)
{
    if (!std::isnan(setting.motion_blur_amount))
        obj.MotionBlurAmount = setting.motion_blur_amount;
    if (setting.auto_exposure_method >= 0)
        obj.AutoExposureMethod = Utils::toEnum<EAutoExposureMethod>(setting.auto_exposure_method);       
    if (!std::isnan(setting.auto_exposure_speed))
        obj.AutoExposureSpeedDown = obj.AutoExposureSpeedUp = setting.auto_exposure_speed;
    if (!std::isnan(setting.auto_exposure_max_brightness))
        obj.AutoExposureMaxBrightness = setting.auto_exposure_max_brightness;
    if (!std::isnan(setting.auto_exposure_min_brightness))
        obj.AutoExposureMinBrightness = setting.auto_exposure_min_brightness;
    if (!std::isnan(setting.auto_exposure_bias))
        obj.AutoExposureBias = setting.auto_exposure_bias;
    if (!std::isnan(setting.auto_exposure_low_percent))
        obj.AutoExposureLowPercent = setting.auto_exposure_low_percent;        
    if (!std::isnan(setting.auto_exposure_high_percent))
        obj.AutoExposureHighPercent = setting.auto_exposure_high_percent;    
    if (!std::isnan(setting.auto_exposure_histogram_log_min))
        obj.HistogramLogMin = setting.auto_exposure_histogram_log_min;    
    if (!std::isnan(setting.auto_exposure_histogram_log_max))
        obj.HistogramLogMax = setting.auto_exposure_histogram_log_max;  
}

void APIPCamera::setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings)
{
    if (!settings.Enabled)
        return;

    UMaterialInstanceDynamic* noise_material_ = UMaterialInstanceDynamic::Create(noise_material_static_, outer);
    noise_materials_[image_type + 1] = noise_material_;


    noise_material_->SetScalarParameterValue("HorzWaveStrength", settings.HorzWaveStrength);
    noise_material_->SetScalarParameterValue("RandSpeed", settings.RandSpeed);
    noise_material_->SetScalarParameterValue("RandSize", settings.RandSize);
    noise_material_->SetScalarParameterValue("RandDensity", settings.RandDensity);
    noise_material_->SetScalarParameterValue("RandContrib", settings.RandContrib);
    noise_material_->SetScalarParameterValue("HorzWaveContrib", settings.HorzWaveContrib);
    noise_material_->SetScalarParameterValue("HorzWaveVertSize", settings.HorzWaveVertSize);
    noise_material_->SetScalarParameterValue("HorzWaveScreenSize", settings.HorzWaveScreenSize);
    noise_material_->SetScalarParameterValue("HorzNoiseLinesContrib", settings.HorzNoiseLinesContrib);
    noise_material_->SetScalarParameterValue("HorzNoiseLinesDensityY", settings.HorzNoiseLinesDensityY);
    noise_material_->SetScalarParameterValue("HorzNoiseLinesDensityXY", settings.HorzNoiseLinesDensityXY);
    noise_material_->SetScalarParameterValue("HorzDistortionStrength", settings.HorzDistortionStrength);
    noise_material_->SetScalarParameterValue("HorzDistortionContrib", settings.HorzDistortionContrib);

    obj.AddBlendable(noise_material_, 1.0f);
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
    //APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    //if (controller && controller->GetViewTarget() == this)
    //    controller->SetViewTarget(nullptr);
}





#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraActor.h"
#include "controllers/VehicleCameraBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "PIPCamera.generated.h"


UCLASS()
class AIRSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    
public:
    struct CaptureSettings {
        //below settinsg are obtained by using Unreal console command (press ~):
        // ShowFlag.VisualizeHDR 1.
        //to replicate camera settings to SceneCapture2D, except motion blur
        typedef msr::airlib::Utils Utils;
        static constexpr float kSceneTargetGamma = Utils::nan<float>(); //1.0f;

        unsigned int width = 256, height = 144; //960 X 540
        float fov_degrees = Utils::nan<float>(); //90.0f
        float auto_exposure_speed = Utils::nan<float>(); // 100.0f;
        float auto_exposure_bias = Utils::nan<float>(); // 0;
        float auto_exposure_max_brightness = Utils::nan<float>(); // 0.64f;
        float auto_exposure_min_brightness = Utils::nan<float>(); // 0.03f;
        float auto_exposure_low_percent = Utils::nan<float>(); // 80.0f;
        float auto_exposure_high_percent = Utils::nan<float>(); // 98.3f;
        float auto_exposure_histogram_log_min = Utils::nan<float>(); // -8;
        float auto_exposure_histogram_log_max = Utils::nan<float>(); // 4;
        float motion_blur_amount = 0;
        float target_gamma = Utils::nan<float>(); //1.0f; //should be defaulted to kSceneTargetGamma for scene
    };


public:
    typedef msr::airlib::VehicleCameraBase::ImageType ImageType;

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();

    void setCameraTypeEnabled(ImageType type, bool enabled);
    bool getCameraTypeEnabled(ImageType type) const;

    USceneCaptureComponent2D* getCaptureComponent(const ImageType type, bool if_active);
    UTextureRenderTarget2D* getRenderTarget(const ImageType type, bool if_active);

    const CaptureSettings& getCaptureSettings(ImageType type);
    void setCaptureSettings(ImageType type, const CaptureSettings& settings);
    
private:
    typedef common_utils::Utils Utils;

    UPROPERTY() TArray<USceneCaptureComponent2D*> captures_;
    UPROPERTY() TArray<UTextureRenderTarget2D*> render_targets_;

    UPROPERTY() UCameraComponent*  camera_;

    std::vector<bool> camera_type_enabled_;

    std::vector<CaptureSettings> capture_settings_;

private:
    static unsigned int imageTypeCount();
    void enableCaptureComponent(const ImageType type, bool is_enabled);
    static void updateCaptureComponentSettings(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, const CaptureSettings& settings);
};

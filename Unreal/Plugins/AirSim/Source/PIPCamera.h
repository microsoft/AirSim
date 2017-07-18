#pragma once

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
        float auto_exposure_speed = 100.0f;
        float auto_exposure_bias = 0;
        float auto_exposure_max_brightness = 0.64f;
        float auto_exposure_min_brightness = 0.03f;
        float auto_exposure_low_percent = 80.0f;
        float auto_exposure_high_percent = 98.3f;      
        float auto_exposure_histogram_log_min = -8;
        float auto_exposure_histogram_log_max = 4;
        float motion_blur_amount = 0;
        float target_gamma = Utils::nan<float>(); //1.0f; //should be defaulted to kSceneTargetGamma for scene
    };


public:
    typedef msr::airlib::VehicleCameraBase::ImageType ImageType;
    typedef msr::airlib::VehicleCameraBase::ImageType_ ImageType_;


    const ImageType DefaultEnabledCameras = ImageType_::None;

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();

    ImageType toggleEnableCameraTypes(ImageType types);
    void setEnableCameraTypes(ImageType types);
    ImageType getEnableCameraTypes();

    USceneCaptureComponent2D* getCaptureComponent(const ImageType type, bool if_active);
    UTextureRenderTarget2D* getRenderTarget(const ImageType type, bool if_active);

    CaptureSettings getCaptureSettings(ImageType_ type);
    void setCaptureSettings(ImageType_ type, const CaptureSettings& settings);
    
private:
    UPROPERTY() USceneCaptureComponent2D* screen_capture_;
    UPROPERTY() USceneCaptureComponent2D* depth_capture_;
    UPROPERTY() USceneCaptureComponent2D* seg_capture_;
    UPROPERTY() UCameraComponent*  camera_;
    UPROPERTY() UTextureRenderTarget2D* scene_render_target_;
    UPROPERTY() UTextureRenderTarget2D* depth_render_target_;
    UPROPERTY() UTextureRenderTarget2D* seg_render_target_;

    ImageType enabled_camera_types_ = DefaultEnabledCameras;

    CaptureSettings scene_capture_settings_, seg_capture_settings_, depth_capture_settings_;

private:
    void enableCaptureComponent(const ImageType type, bool is_enabled);
    static void updateCaptureComponentSettings(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, const CaptureSettings& settings);
};

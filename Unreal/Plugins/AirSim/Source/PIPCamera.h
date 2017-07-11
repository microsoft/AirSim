#pragma once

#include "Camera/CameraActor.h"
#include "controllers/VehicleCameraBase.hpp"
#include "PIPCamera.generated.h"


UCLASS()
class AIRSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    
public:
    struct CaptureSettings {
        unsigned int width = 256, height = 144; //960 X 540
        float fov_degrees = 90;
        float auto_exposure_speed = 100.0f;
        float motion_blur_amount = 0.0f;
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

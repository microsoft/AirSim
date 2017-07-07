#pragma once

#include "Camera/CameraActor.h"
#include "controllers/VehicleCameraBase.hpp"
#include "PIPCamera.generated.h"


UCLASS()
class AIRSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    
    
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

private:
    UPROPERTY() USceneCaptureComponent2D* screen_capture_;
    UPROPERTY() USceneCaptureComponent2D* depth_capture_;
    UPROPERTY() USceneCaptureComponent2D* seg_capture_;
    UPROPERTY() UCameraComponent*  camera_;
    UPROPERTY() UTextureRenderTarget2D* scene_render_target_;
    UPROPERTY() UTextureRenderTarget2D* depth_render_target_;
    UPROPERTY() UTextureRenderTarget2D* seg_render_target_;

    ImageType enabled_camera_types_ = DefaultEnabledCameras;

private:
    void enableCaptureComponent(const ImageType type, bool is_enabled);
};

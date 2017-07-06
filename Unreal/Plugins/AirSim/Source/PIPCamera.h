#pragma once

#include "Camera/CameraActor.h"
#include "PIPCamera.generated.h"


UENUM(BlueprintType, meta=(Bitflags))
enum class EPIPCameraType : uint8
{
    PIP_CAMERA_TYPE_NONE = 0	UMETA(DisplayName="None"),
    PIP_CAMERA_TYPE_SCENE = 1	UMETA(DisplayName="Scene"),
    PIP_CAMERA_TYPE_DEPTH = 2	UMETA(DisplayName="Depth"),
    PIP_CAMERA_TYPE_SEG = 4 	UMETA(DisplayName="Segmentation"),
    PIP_CAMERA_TYPE_ALL = 127     UMETA(DisplayName="All")
};
ENUM_CLASS_FLAGS(EPIPCameraType)


UCLASS()
class AIRSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    
    
public:
    static constexpr EPIPCameraType DefaultEnabledCameras = EPIPCameraType::PIP_CAMERA_TYPE_NONE;

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();

    EPIPCameraType toggleEnableCameraTypes(EPIPCameraType types);
    void setEnableCameraTypes(EPIPCameraType types);
    EPIPCameraType getEnableCameraTypes();

    USceneCaptureComponent2D* getCaptureComponent(const EPIPCameraType type, bool if_active);
    UTextureRenderTarget2D* getRenderTarget(const EPIPCameraType type, bool if_active);

private:
    UPROPERTY() USceneCaptureComponent2D* screen_capture_;
    UPROPERTY() USceneCaptureComponent2D* depth_capture_;
    UPROPERTY() USceneCaptureComponent2D* seg_capture_;
    UPROPERTY() UCameraComponent*  camera_;
    UPROPERTY() UTextureRenderTarget2D* scene_render_target_;
    UPROPERTY() UTextureRenderTarget2D* depth_render_target_;
    UPROPERTY() UTextureRenderTarget2D* seg_render_target_;

    EPIPCameraType enabled_camera_types_ = DefaultEnabledCameras;

private:
    void enableCaptureComponent(const EPIPCameraType type, bool is_enabled);
};

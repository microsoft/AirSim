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

UENUM(BlueprintType)
enum class EPIPCameraMode : uint8
{
    PIP_CAMERA_MODE_NONE = 0	UMETA(DisplayName="None"),
    PIP_CAMERA_MODE_MAIN = 1	UMETA(DisplayName="Main"),
    PIP_CAMERA_MODE_PIP = 2	UMETA(DisplayName="PIP")
};

UCLASS()
class AIRSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    
    
public:
    static constexpr EPIPCameraType DefaultEnabledCameras = EPIPCameraType::PIP_CAMERA_TYPE_NONE;
    static constexpr EPIPCameraMode DefaultCameraMode = EPIPCameraMode::PIP_CAMERA_MODE_NONE;

    APIPCamera();
    virtual void PostInitializeComponents() override;

    UFUNCTION(BlueprintCallable, Category = "Switching")
    void setToMainView();
    UFUNCTION(BlueprintCallable, Category = "Switching")
    void setToPIPView();
    UFUNCTION(BlueprintCallable, Category = "Switching")
    void disableAll();

    UFUNCTION(BlueprintCallable, Category = "Cameras")
    EPIPCameraType toggleEnableCameraTypes(EPIPCameraType types);
    UFUNCTION(BlueprintCallable, Category = "Cameras")
    void setEnableCameraTypes(EPIPCameraType types);
    UFUNCTION(BlueprintCallable, Category = "Cameras")
    EPIPCameraType getEnableCameraTypes();
    UFUNCTION(BlueprintCallable, Category = "Cameras")
    EPIPCameraMode getCameraMode();

	USceneCaptureComponent2D* getCaptureComponent(const EPIPCameraType type, bool if_active);

private:
    UPROPERTY() USceneCaptureComponent2D* screen_capture_;
    UPROPERTY() USceneCaptureComponent2D* depth_capture_;
    UPROPERTY() USceneCaptureComponent2D* seg_capture_;
    UPROPERTY() UCameraComponent*  camera_;
    UPROPERTY() UTextureRenderTarget2D* scene_render_target_;
    UPROPERTY() UTextureRenderTarget2D* depth_render_target_;
    UPROPERTY() UTextureRenderTarget2D* seg_render_target_;

    //UPROPERTY(BlueprintReadWrite, Category = "Cameras")
    EPIPCameraMode camera_mode_ = DefaultCameraMode;   

    //UPROPERTY(BlueprintReadWrite, Category = "Cameras", meta = (Bitmask, BitmaskEnum = "EPIPCameraType"))
    EPIPCameraType enabled_camera_types_ = DefaultEnabledCameras;

private:
    void activateCaptureComponent(const EPIPCameraType type);
    void deactivateCaptureComponent(const EPIPCameraType type);
    void deactivateMain();
    void deactivatePIP();
    void refreshCurrentMode();
    UTextureRenderTarget2D* getTextureRenderTarget(const EPIPCameraType type, bool if_active);
};

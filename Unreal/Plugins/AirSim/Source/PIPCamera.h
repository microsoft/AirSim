#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"
#include "Runtime/Core/Public/PixelFormat.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/AirSimSettings.hpp"
#include "NedTransform.h"
#include "DetectionComponent.h"

//CinemAirSim
#include <CineCameraActor.h>
#include <CineCameraComponent.h>

#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PIPCamera.generated.h"

UCLASS()
class AIRSIM_API APIPCamera : public ACineCameraActor //CinemAirSim
{
    GENERATED_BODY()

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;
    typedef AirSimSettings::CameraSetting CameraSetting;

    APIPCamera(const FObjectInitializer& ObjectInitializer); //CinemAirSim

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();
    void onViewModeChanged(bool nodisplay);

    //CinemAirSim methods
    std::vector<std::string> getPresetLensSettings() const;
    void setPresetLensSettings(std::string preset_string);
    std::vector<std::string> getPresetFilmbackSettings() const;
    void setPresetFilmbackSettings(std::string preset_string);
    std::string getLensSettings() const;
    std::string getFilmbackSettings() const;
    float setFilmbackSettings(float sensor_width, float sensor_height);
    float getFocalLength() const;
    void setFocalLength(float focal_length);
    void enableManualFocus(bool enable);
    float getFocusDistance() const;
    void setFocusDistance(float focus_distance);
    float getFocusAperture() const;
    void setFocusAperture(float focus_aperture);
    void enableFocusPlane(bool enable);
    std::string getCurrentFieldOfView() const;
    //end CinemAirSim methods

    void setCameraTypeEnabled(ImageType type, bool enabled);
    bool getCameraTypeEnabled(ImageType type) const;
    void setCaptureUpdate(USceneCaptureComponent2D* capture, bool nodisplay);
    void setCameraTypeUpdate(ImageType type, bool nodisplay);

    void setupCameraFromSettings(const CameraSetting& camera_setting, const NedTransform& ned_transform);
    void setCameraPose(const msr::airlib::Pose& relative_pose);
    void setCameraPitch(float pitch);
    void setCameraRoll(float roll);
    void setCameraYaw(float yaw);
    void setCameraFoV(float fov_degrees);
    msr::airlib::CameraInfo getCameraInfo() const;
    std::vector<float> getDistortionParams() const;
    void setDistortionParam(const std::string& param_name, float value);

    msr::airlib::ProjectionMatrix getProjectionMatrix(const ImageType image_type) const;

    USceneCaptureComponent2D* getCaptureComponent(const ImageType type, bool if_active);
    UTextureRenderTarget2D* getRenderTarget(const ImageType type, bool if_active);
    UDetectionComponent* getDetectionComponent(const ImageType type, bool if_active) const;

    msr::airlib::Pose getPose() const;

private: //members
    UPROPERTY()
    UMaterialParameterCollection* distortion_param_collection_;
    UPROPERTY()
    UMaterialParameterCollectionInstance* distortion_param_instance_;

    UPROPERTY()
    TArray<USceneCaptureComponent2D*> captures_;
    UPROPERTY()
    TArray<UTextureRenderTarget2D*> render_targets_;
    UPROPERTY()
    TArray<UDetectionComponent*> detections_;

    //CinemAirSim
    UPROPERTY()
    UCineCameraComponent* camera_;
    //TMap<int, UMaterialInstanceDynamic*> noise_materials_;
    //below is needed because TMap doesn't work with UPROPERTY, but we do have -ve index
    UPROPERTY()
    TArray<UMaterialInstanceDynamic*> noise_materials_;
    UPROPERTY()
    TArray<UMaterialInstanceDynamic*> distortion_materials_;
    UPROPERTY()
    UMaterial* noise_material_static_;
    UPROPERTY()
    UMaterial* distortion_material_static_;

    std::vector<bool> camera_type_enabled_;
    FRotator gimbald_rotator_;
    float gimbal_stabilization_;
    const NedTransform* ned_transform_;
    TMap<int, EPixelFormat> image_type_to_pixel_format_map_;

    FObjectFilter object_filter_;

private: //methods
    typedef common_utils::Utils Utils;
    typedef AirSimSettings::CaptureSetting CaptureSetting;
    typedef AirSimSettings::NoiseSetting NoiseSetting;

    static unsigned int imageTypeCount();
    void enableCaptureComponent(const ImageType type, bool is_enabled);
    static void updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target,
                                              bool auto_format, const EPixelFormat& pixel_format, const CaptureSetting& setting, const NedTransform& ned_transform,
                                              bool force_linear_gamma);
    void setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings);
    void setDistortionMaterial(int image_type, UObject* outer, FPostProcessSettings& obj);
    static void updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting);
    //CinemAirSim
    static void updateCameraSetting(UCineCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform);
    void copyCameraSettingsToAllSceneCapture(UCameraComponent* camera);
    void copyCameraSettingsToSceneCapture(UCameraComponent* src, USceneCaptureComponent2D* dst);
    //end CinemAirSim
};

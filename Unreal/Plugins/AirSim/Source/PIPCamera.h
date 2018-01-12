#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraActor.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/AirSimSettings.hpp"
#include "PIPCamera.generated.h"


UCLASS()
class AIRSIM_API APIPCamera : public ACameraActor
{
    GENERATED_BODY()
    

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;
    typedef AirSimSettings::CaptureSetting CaptureSetting;
    typedef AirSimSettings::NoiseSetting NoiseSetting;


    APIPCamera();

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();

    void setCameraTypeEnabled(ImageType type, bool enabled);
    bool getCameraTypeEnabled(ImageType type) const;
    void setImageTypeSettings(int image_type, const APIPCamera::CaptureSetting& capture_setting,
        const APIPCamera::NoiseSetting& noise_setting);

    USceneCaptureComponent2D* getCaptureComponent(const ImageType type, bool if_active);
    UTextureRenderTarget2D* getRenderTarget(const ImageType type, bool if_active);

    
private:
    typedef common_utils::Utils Utils;


    UPROPERTY() TArray<USceneCaptureComponent2D*> captures_;
    UPROPERTY() TArray<UTextureRenderTarget2D*> render_targets_;

    UPROPERTY() UCameraComponent*  camera_;
    UPROPERTY() UMaterialInstanceDynamic* noise_material_ = nullptr;
    UPROPERTY() UMaterial* noise_material_static_ = nullptr;

    std::vector<bool> camera_type_enabled_;

private:
    static unsigned int imageTypeCount();
    void enableCaptureComponent(const ImageType type, bool is_enabled);
    static void updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, const CaptureSetting& setting);
    void setNoiseMaterial(FPostProcessSettings& obj, const NoiseSetting& settings);
};

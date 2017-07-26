#pragma once

#include "CoreMinimal.h"
#include "VehiclePawnBase.h"
#include "PIPCamera.h"
#include "GameFramework/Actor.h"
#include "ManualPoseController.h"
#include "CameraDirector.generated.h"


UENUM(BlueprintType)
enum class ECameraDirectorMode : uint8
{
    CAMERA_DIRECTOR_MODE_FPV = 1	UMETA(DisplayName="FPV"),
    CAMERA_DIRECTOR_MODE_GROUND_OBSERVER = 2	UMETA(DisplayName="GroundObserver"),
    CAMERA_DIRECTOR_MODE_FLY_WITH_ME = 3	UMETA(DisplayName="FlyWithMe"),
    CAMERA_DIRECTOR_MODE_MANUAL = 4	UMETA(DisplayName="Manual")
};

UCLASS()
class AIRSIM_API ACameraDirector : public AActor
{
    GENERATED_BODY()
    
public:
    void inputEventFpvView();
    void inputEventGroundView();
    void inputEventManualView();
    void inputEventFlyWithView();

public:	
    ACameraDirector();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick( float DeltaSeconds ) override;

    UFUNCTION(BlueprintCallable, Category = "Modes")
    ECameraDirectorMode getMode();
    UFUNCTION(BlueprintCallable, Category = "Modes")
    void setMode(ECameraDirectorMode mode);

    void initializeForBeginPlay(ECameraDirectorMode view_mode, AVehiclePawnBase* vehicle, APIPCamera* external_camera);

    void setCameras(APIPCamera* external_camera, AVehiclePawnBase* vehicle);
    APIPCamera* getFpvCamera() const;
    APIPCamera* getExternalCamera() const;

private:
    void setupInputBindings();	

private:
    APIPCamera* fpv_camera_;
    APIPCamera* external_camera_;
    AActor* follow_actor_;

    ECameraDirectorMode mode_;
    UPROPERTY() UManualPoseController* manual_pose_controller_;

    FVector camera_start_location_;
    FVector initial_ground_obs_offset_;
    FRotator camera_start_rotation_;
    bool ext_obs_fixed_z_;
};

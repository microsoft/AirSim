#pragma once

#include "CoreMinimal.h"
#include "VehiclePawnWrapper.h"
#include "PIPCamera.h"
#include "GameFramework/Actor.h"
#include "ManualPoseController.h"
#include "GameFramework/SpringArmComponent.h"
#include "CameraDirector.generated.h"


UENUM(BlueprintType)
enum class ECameraDirectorMode : uint8
{
    CAMERA_DIRECTOR_MODE_FPV = 1	UMETA(DisplayName="FPV"),
    CAMERA_DIRECTOR_MODE_GROUND_OBSERVER = 2	UMETA(DisplayName="GroundObserver"),
    CAMERA_DIRECTOR_MODE_FLY_WITH_ME = 3	UMETA(DisplayName="FlyWithMe"),
    CAMERA_DIRECTOR_MODE_MANUAL = 4	UMETA(DisplayName="Manual"),
    CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE = 5	UMETA(DisplayName = "SpringArmChase")
};

UCLASS()
class AIRSIM_API ACameraDirector : public AActor
{
    GENERATED_BODY()

public:
    /** Spring arm that will offset the camera */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    USpringArmComponent* SpringArm;

public:
    void inputEventFpvView();
    void inputEventGroundView();
    void inputEventManualView();
    void inputEventFlyWithView();
    void inputEventSpringArmChaseView();

public:	
    ACameraDirector();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick( float DeltaSeconds ) override;

    UFUNCTION(BlueprintCallable, Category = "Modes")
    ECameraDirectorMode getMode();
    UFUNCTION(BlueprintCallable, Category = "Modes")
    void setMode(ECameraDirectorMode mode);

    void initializeForBeginPlay(ECameraDirectorMode view_mode, VehiclePawnWrapper* vehicle_pawn_wrapper, APIPCamera* external_camera);

    void setCameras(APIPCamera* external_camera, VehiclePawnWrapper* vehicle_pawn_wrapper);
    APIPCamera* getFpvCamera() const;
    APIPCamera* getExternalCamera() const;

private:
    void setupInputBindings();	
    void attachSpringArm(bool attach);


private:
    APIPCamera* fpv_camera_;
    APIPCamera* external_camera_;
    AActor* follow_actor_;

    USceneComponent* last_parent_ = nullptr;

    ECameraDirectorMode mode_;
    UPROPERTY() UManualPoseController* manual_pose_controller_;

    FVector camera_start_location_;
    FVector initial_ground_obs_offset_;
    FRotator camera_start_rotation_;
    bool ext_obs_fixed_z_;
};

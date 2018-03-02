#pragma once

#include "CoreMinimal.h"
#include "VehiclePawnWrapper.h"
#include "PIPCamera.h"
#include "GameFramework/Actor.h"
#include "ManualPoseController.h"
#include "common/common_utils/Utils.hpp"
#include "GameFramework/SpringArmComponent.h"
#include "CameraDirector.generated.h"


UENUM(BlueprintType)
enum class ECameraDirectorMode : uint8
{
    CAMERA_DIRECTOR_MODE_FPV = 1	UMETA(DisplayName = "FPV"),
    CAMERA_DIRECTOR_MODE_GROUND_OBSERVER = 2	UMETA(DisplayName = "GroundObserver"),
    CAMERA_DIRECTOR_MODE_FLY_WITH_ME = 3	UMETA(DisplayName = "FlyWithMe"),
    CAMERA_DIRECTOR_MODE_MANUAL = 4	UMETA(DisplayName = "Manual"),
    CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE = 5	UMETA(DisplayName = "SpringArmChase"),
    CAMERA_DIRECTOR_MODE_BACKUP = 6     UMETA(DisplayName = "Backup"),
    CAMERA_DIRECTOR_MODE_NODISPLAY = 7      UMETA(DisplayName = "No Display")
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
    void inputEventBackupView();
    void inputEventNoDisplayView();

public:
    ACameraDirector();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    UFUNCTION(BlueprintCallable, Category = "Modes")
        ECameraDirectorMode getMode();
    UFUNCTION(BlueprintCallable, Category = "Modes")
        void setMode(ECameraDirectorMode mode);

    void initializeForBeginPlay(ECameraDirectorMode view_mode, VehiclePawnWrapper* vehicle_pawn_wrapper, APIPCamera* external_camera);

    void setCameras(APIPCamera* external_camera, VehiclePawnWrapper* vehicle_pawn_wrapper);
    APIPCamera* getFpvCamera() const;
    APIPCamera* getExternalCamera() const;
    APIPCamera* getBackupCamera() const;
    void setFollowDistance(const int follow_distance) { this->follow_distance_ = follow_distance; }
    void setCameraRotationLagEnabled(const bool lag_enabled) { this->camera_rotation_lag_enabled_ = lag_enabled; }
    void setFpvCameraIndex(const int fpv_camera_index) { this->fpv_camera_index_ = fpv_camera_index; }

    // Both of these get bound to the 'b' key
    //
    void setBackupCameraIndex(const int backup_camera_index) { this->backup_camera_index_ = backup_camera_index; }
    void enableFlyWithMeMode() { this->backup_camera_index_ = -1; }

private:
    void setupInputBindings();
    void attachSpringArm(bool attach);
    void disableCameras(bool fpv, bool backup, bool external);
    void setupCameraFromSettings();


private:
    typedef common_utils::Utils Utils;


    APIPCamera* fpv_camera_;
    APIPCamera* backup_camera_;
    APIPCamera* external_camera_;
    AActor* follow_actor_;

    USceneComponent* last_parent_ = nullptr;

    ECameraDirectorMode mode_;
    UPROPERTY() UManualPoseController* manual_pose_controller_;

    FVector camera_start_location_;
    FVector initial_ground_obs_offset_;
    FRotator camera_start_rotation_;
    bool ext_obs_fixed_z_;
    int follow_distance_;
    bool camera_rotation_lag_enabled_;
    int fpv_camera_index_;
    int backup_camera_index_ = 4;
};

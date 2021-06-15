#pragma once

#include "CoreMinimal.h"
#include "UObject/ConstructorHelpers.h"

#include "physics/Kinematics.hpp"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "PawnEvents.h"
#include "PIPCamera.h"
#include "ManualPoseController.h"

#include "ComputerVisionPawn.generated.h"

UCLASS()
class AComputerVisionPawn : public APawn
{
    GENERATED_BODY()

public:
    AComputerVisionPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                           FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface
    void initializeForBeginPlay();
    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
    PawnEvents* getPawnEvents()
    {
        return &pawn_events_;
    }

private:
    UPROPERTY()
    UClass* pip_camera_class_;

    PawnEvents pawn_events_;

    UPROPERTY()
    USceneComponent* camera_front_center_base_;
    UPROPERTY()
    USceneComponent* camera_front_left_base_;
    UPROPERTY()
    USceneComponent* camera_front_right_base_;
    UPROPERTY()
    USceneComponent* camera_bottom_center_base_;
    UPROPERTY()
    USceneComponent* camera_back_center_base_;

    UPROPERTY()
    APIPCamera* camera_front_center_;
    UPROPERTY()
    APIPCamera* camera_front_left_;
    UPROPERTY()
    APIPCamera* camera_front_right_;
    UPROPERTY()
    APIPCamera* camera_bottom_center_;
    UPROPERTY()
    APIPCamera* camera_back_center_;

    UPROPERTY()
    UManualPoseController* manual_pose_controller_;
};

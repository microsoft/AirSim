#pragma once

#include "GameFramework/RotatingMovementComponent.h"

#include <memory>
#include "PIPCamera.h"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "MultirotorPawnEvents.h"

#include "FlyingPawn.generated.h"

UCLASS()
class AIRSIM_API AFlyingPawn : public APawn
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
    float RotatorFactor = 1.0f;

    AFlyingPawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface
    void initializeForBeginPlay();
    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
    MultirotorPawnEvents* getPawnEvents()
    {
        return &pawn_events_;
    }
    //called by API to set rotor speed
    void setRotorSpeed(const std::vector<MultirotorPawnEvents::RotorActuatorInfo>& rotor_infos);
    void initializeRotors(const std::vector<MultirotorPawnEvents::RotorActuatorInfo>& rotor_infos);


private: //variables
    //Unreal components
    UPROPERTY() APIPCamera* camera_front_left_;
    UPROPERTY() APIPCamera* camera_front_right_;
    UPROPERTY() APIPCamera* camera_front_center_;
    UPROPERTY() APIPCamera* camera_back_center_;
    UPROPERTY() APIPCamera* camera_bottom_center_;

    UPROPERTY() TArray<URotatingMovementComponent*> rotating_movements_;

    MultirotorPawnEvents pawn_events_;
    int init_id_;
};

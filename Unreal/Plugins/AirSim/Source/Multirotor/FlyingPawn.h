#pragma once

#include "GameFramework/RotatingMovementComponent.h"
#include <memory>
#include "PIPCamera.h"
#include "common/common_utils/Signal.hpp"
#include "FlyingPawn.generated.h"

UCLASS()
class AIRSIM_API AFlyingPawn : public APawn
{
    GENERATED_BODY()

public:
    typedef common_utils::Signal<UPrimitiveComponent*, AActor*, UPrimitiveComponent*, bool, FVector,
        FVector, FVector, const FHitResult&> CollisionSignal;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
    float RotatorFactor = 1.0f;

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface
    void initializeForBeginPlay();
    std::map<std::string, APIPCamera*> getCameras() const;
    CollisionSignal& getCollisionSignal()
    {
        return collision_signal_;
    }
    //called by API to set rotor speed
    void setRotorSpeed(int rotor_index, float radsPerSec);


private: //variables
    //Unreal components
    static constexpr size_t rotor_count = 4;
    UPROPERTY() APIPCamera* camera_front_left_;
    UPROPERTY() APIPCamera* camera_front_right_;
    UPROPERTY() APIPCamera* camera_front_center_;
    UPROPERTY() APIPCamera* camera_back_center_;
    UPROPERTY() APIPCamera* camera_bottom_center_;

    UPROPERTY() URotatingMovementComponent* rotating_movements_[rotor_count];

    CollisionSignal collision_signal_;
};

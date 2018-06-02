#pragma once

#include "VehicleSimApi.h"
#include "GameFramework/RotatingMovementComponent.h"
#include <memory>
#include "vehicles/multirotor/api/MultirotorCommon.hpp"
#include "PIPCamera.h"
#include "api/VehicleSimApiBase.hpp"
#include "FlyingPawn.generated.h"

UCLASS()
class AIRSIM_API AFlyingPawn : public APawn
{
    GENERATED_BODY()

public:
    AFlyingPawn();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
    float RotatorFactor = 1.0f;

    void setRotorSpeed(int rotor_index, float radsPerSec);
    void initializeForBeginPlay(const NedTransform& global_transform, const std::string& vehicle_name);

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    msr::airlib::VehicleSimApiBase* getVehicleSimApi()
    {
        return vehicle_sim_api_.get();
    }

private: //variables
    UPROPERTY() UClass* pip_camera_class_;

         //Unreal components
    static constexpr size_t rotor_count = 4;
    UPROPERTY() APIPCamera* fpv_camera_front_left_;
    UPROPERTY() APIPCamera* fpv_camera_front_right_;
    UPROPERTY() APIPCamera* fpv_camera_front_center_;
    UPROPERTY() APIPCamera* fpv_camera_back_center_;
    UPROPERTY() APIPCamera* fpv_camera_bottom_center_;

    UPROPERTY() URotatingMovementComponent* rotating_movements_[rotor_count];

    std::map<std::string, APIPCamera*> cameras_;
    std::_Unique_ptr_base<msr::airlib::VehicleSimApiBase> vehicle_sim_api_;
};

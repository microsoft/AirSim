#include "FlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "MultiRotorConnector.h"
#include "common/Common.hpp"


AFlyingPawn::AFlyingPawn()
{
    wrapper_.reset(new VehiclePawnWrapper());
}

void AFlyingPawn::initializeForBeginPlay()
{
    //get references of components so we can use later
    setupComponentReferences();

    std::vector<APIPCamera*> cameras = {fpv_camera_front_center_, fpv_camera_front_right_, fpv_camera_front_left_, 
        fpv_camera_bottom_center_, fpv_camera_back_center_};
    wrapper_->initialize(this, cameras);
}

void AFlyingPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    wrapper_->onCollision(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

VehiclePawnWrapper* AFlyingPawn::getVehiclePawnWrapper()
{
    return wrapper_.get();
}

void AFlyingPawn::setRotorSpeed(int rotor_index, float radsPerSec)
{
    if (rotor_index >= 0 && rotor_index < rotor_count) {
        auto comp = rotating_movements_[rotor_index];
        if (comp != nullptr) {
            comp->RotationRate.Yaw = radsPerSec * 180.0f / M_PIf * RotatorFactor;
        }
    }
}

void AFlyingPawn::setupComponentReferences()
{
    fpv_camera_front_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontRightCamera")))->GetChildActor());
    fpv_camera_front_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontLeftCamera")))->GetChildActor());
    fpv_camera_front_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    fpv_camera_back_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    fpv_camera_bottom_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());

    for (auto i = 0; i < rotor_count; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }
}

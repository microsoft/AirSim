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

    std::vector<APIPCamera*> cameras = {fpv_camera_right_, fpv_camera_left_};
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
    fpv_camera_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("RightPIPCamera")))->GetChildActor());
    fpv_camera_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("LeftPIPCamera")))->GetChildActor());

    for (auto i = 0; i < rotor_count; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }
}

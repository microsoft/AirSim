#include "FlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "MultiRotorConnector.h"
#include "common/Common.hpp"

void AFlyingPawn::initialize()
{
    Super::initialize();
}

void AFlyingPawn::initializeForBeginPlay()
{
    Super::initializeForBeginPlay();

    //get references of components so we can use later
    setupComponentReferences();

    //set stencil IDs
    setStencilIDs();

    setupInputBindings();

    setupCamerasFromSettings();
}

void AFlyingPawn::setStencilIDs()
{
    TArray<AActor*> foundActors;
    UAirBlueprintLib::FindAllActor<AActor>(this, foundActors);
    TArray<UStaticMeshComponent*> components;
    int stencil = 0;
    for (AActor* actor : foundActors) {
        actor->GetComponents(components);
        if (components.Num() == 1) {
            components[0]->SetRenderCustomDepth(true);
            components[0]->CustomDepthStencilValue = (stencil++) % 256;
            components[0]->MarkRenderStateDirty();
        }
    }
}

void AFlyingPawn::reset()
{
    Super::reset();
}


APIPCamera* AFlyingPawn::getCamera(int index)
{
    switch (index) {
    case 0: return fpv_camera_right_; 
    case 1: return fpv_camera_left_; 
    default:
        UAirBlueprintLib::LogMessageString("FlyingPawn doesn't have camera with index = ", std::to_string(index), LogDebugLevel::Failure);
        return fpv_camera_right_;
    }
}

int AFlyingPawn::getCameraCount()
{
    return 2;
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


void AFlyingPawn::setupInputBindings()
{
    //UAirBlueprintLib::EnableInput(this);
}
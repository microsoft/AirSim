#include "AirSim.h"
#include "FlyingPawn.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "MultiRotorConnector.h"
#include "common/Common.hpp"

void AFlyingPawn::initialize()
{
    Super::initialize();
}

//this gets called from Blueprint
void AFlyingPawn::initializeForPlay()
{
    //get references of components so we can use later
    setupComponentReferences();

    //set stencil IDs
    setStencilIDs();

    setupInputBindings();
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


APIPCamera* AFlyingPawn::getFpvCamera()
{
    return fpv_camera_;
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

std::string AFlyingPawn::getVehicleName()
{
    return std::string(TCHAR_TO_UTF8(*VehicleName));
}

void AFlyingPawn::setupComponentReferences()
{
    fpv_camera_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("LeftPIPCamera")))->GetChildActor());

    for (auto i = 0; i < 4; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }
}


void AFlyingPawn::setupInputBindings()
{
    //UAirBlueprintLib::EnableInput(this);
}
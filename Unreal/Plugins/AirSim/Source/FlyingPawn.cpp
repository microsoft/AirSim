#include "AirSim.h"
#include "FlyingPawn.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "MultiRotorConnector.h"

void AFlyingPawn::initialize()
{
    Super::initialize();
}

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

const AFlyingPawn::RCData& AFlyingPawn::getRCData()
{
    return rc_data;
}

void AFlyingPawn::reset()
{
    Super::reset();

    rc_data = RCData();
    rc_data.switch1 = rc_data.switch2 = rc_data.switch3 = 1;
}


APIPCamera* AFlyingPawn::getFpvCamera()
{
    return fpv_camera_;
}

void AFlyingPawn::setRotorSpeed(int rotor_index, float radsPerSec)
{
    if (rotor_index >= 0 && rotor_index < rotor_count)
        rotating_movements_[rotor_index]->RotationRate.Yaw = radsPerSec * 180.0f / M_PIf * RotatorFactor;
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

void AFlyingPawn::inputEventThrottle(float val)
{
    rc_data.throttle = val;
    UAirBlueprintLib::LogMessage(TEXT("Throttle: "), FString::SanitizeFloat(val), LogDebugLevel::Informational);
}
void AFlyingPawn::inputEventYaw(float val)
{
    rc_data.yaw = val;
    UAirBlueprintLib::LogMessage(TEXT("Yaw: "), FString::SanitizeFloat(val), LogDebugLevel::Informational);
}
void AFlyingPawn::inputEventPitch(float val)
{
    rc_data.pitch = -val;
    UAirBlueprintLib::LogMessage(TEXT("Pitch: "), FString::SanitizeFloat(val), LogDebugLevel::Informational);
}
void AFlyingPawn::inputEventRoll(float val)
{
    rc_data.roll = val;
    UAirBlueprintLib::LogMessage(TEXT("Roll: "), FString::SanitizeFloat(val), LogDebugLevel::Informational);
}
void AFlyingPawn::inputEventArmDisArm()
{
    rc_data.switch5 = rc_data.switch5 <= 0 ? 1 : 0;
    UAirBlueprintLib::LogMessage(TEXT("Arm/Disarm"), FString::SanitizeFloat(rc_data.switch5), LogDebugLevel::Informational);
}

void AFlyingPawn::setupInputBindings()
{
    this->EnableInput(this->GetWorld()->GetFirstPlayerController());

    UAirBlueprintLib::BindAxisToKey("InputEventThrottle", EKeys::Gamepad_LeftY, this, &AFlyingPawn::inputEventThrottle);
    UAirBlueprintLib::BindAxisToKey("InputEventYaw", EKeys::Gamepad_LeftX, this, &AFlyingPawn::inputEventYaw);
    UAirBlueprintLib::BindAxisToKey("InputEventPitch", EKeys::Gamepad_RightY, this, &AFlyingPawn::inputEventPitch);
    UAirBlueprintLib::BindAxisToKey("InputEventRoll", EKeys::Gamepad_RightX, this, &AFlyingPawn::inputEventRoll);
    UAirBlueprintLib::BindActionToKey("InputEventArmDisArm", EKeys::Gamepad_LeftTrigger, this, &AFlyingPawn::inputEventArmDisArm);

}
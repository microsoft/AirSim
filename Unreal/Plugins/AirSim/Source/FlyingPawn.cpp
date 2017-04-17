#include "AirSim.h"
#include "FlyingPawn.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "MultiRotorConnector.h"
#include "SimJoyStick/SimJoyStick.h"
#include "common/Common.hpp"

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

	detectUsbRc();
}

void AFlyingPawn::detectUsbRc()
{
	std::string vehicleName = AFlyingPawn::getVehicleName();

	if (vehicleName.compare(std::string("Quad1")) == 0)
	{
		joystick_.getJoyStickState(0, joystick_state_);
		rc_data_.is_connected = joystick_state_.is_connected;

		if (rc_data_.is_connected)
			UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB for vehicle 1: "), "Detected", LogDebugLevel::Informational);
		else
			UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB: "), "Not detected", LogDebugLevel::Informational);
	}

	if (vehicleName.compare(std::string("Quad2")) == 0)
	{
		joystick_.getJoyStickState(1, joystick_state_);
		rc_data_.is_connected = joystick_state_.is_connected;

		if (rc_data_.is_connected)
			UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB for vehicle 2: "), "Detected", LogDebugLevel::Informational);
		else
			UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB: "), "Not detected", LogDebugLevel::Informational);
	}

	if (vehicleName.compare(std::string("Quad3")) == 0)
	{
		joystick_.getJoyStickState(2, joystick_state_);
		rc_data_.is_connected = joystick_state_.is_connected;

		if (rc_data_.is_connected)
			UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB for vehicle 3: "), "Detected", LogDebugLevel::Informational);
		else
			UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB for vehicle 3: "), "Not detected", LogDebugLevel::Informational);
	}
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
	std::string vehicleName = AFlyingPawn::getVehicleName();

	if (vehicleName.compare(std::string("Quad1")) == 0)
		joystick_.getJoyStickState(0, joystick_state_);
	else if (vehicleName.compare(std::string("Quad2")) == 0)
		joystick_.getJoyStickState(1, joystick_state_);
	else if (vehicleName.compare(std::string("Quad3")) == 0)
		joystick_.getJoyStickState(2, joystick_state_);

	rc_data_.is_connected = joystick_state_.is_connected;

	if (rc_data_.is_connected) {
		rc_data_.throttle = joyStickToRC(joystick_state_.left_y);
		rc_data_.yaw = joyStickToRC(joystick_state_.left_x);
		rc_data_.roll = joyStickToRC(joystick_state_.right_x);
		rc_data_.pitch = joyStickToRC(joystick_state_.right_y);

		rc_data_.switch1 = joystick_state_.left_trigger ? 1 : 0;
		rc_data_.switch2 = joystick_state_.right_trigger ? 1 : 0;
	}
	//else don't waste time

	return rc_data_;
}

float AFlyingPawn::joyStickToRC(int16_t val)
{
	float valf = static_cast<float>(val);
	return (valf) / (Utils::max<uint16_t>()/2);
}

void AFlyingPawn::reset()
{
	Super::reset();

	rc_data_ = RCData();
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
	//this->EnableInput(this->GetWorld()->GetFirstPlayerController());

	//UAirBlueprintLib::BindAxisToKey("InputEventThrottle", EKeys::Gamepad_LeftY, this, &AFlyingPawn::inputEventThrottle);
	//UAirBlueprintLib::BindAxisToKey("InputEventYaw", EKeys::Gamepad_LeftX, this, &AFlyingPawn::inputEventYaw);
	//UAirBlueprintLib::BindAxisToKey("InputEventPitch", EKeys::Gamepad_RightY, this, &AFlyingPawn::inputEventPitch);
	//UAirBlueprintLib::BindAxisToKey("InputEventRoll", EKeys::Gamepad_RightX, this, &AFlyingPawn::inputEventRoll);
	//UAirBlueprintLib::BindActionToKey("InputEventArmDisArm", EKeys::Gamepad_LeftTrigger, this, &AFlyingPawn::inputEventArmDisArm);
}
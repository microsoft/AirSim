#include "AirSim.h"
#include "FlyingPawn.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "MavMultiRotorConnector.h"

void AFlyingPawn::initialize()
{
	Super::initialize();

	//get references of components so we can use later
	setupComponentReferences();

	//set stencil IDs
	setStencilIDs();
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

APIPCamera* AFlyingPawn::getFpvCamera()
{
	return fpv_camera_;
}

void AFlyingPawn::setRotorSpeed(int rotor_index, float radsPerSec)
{
	if (rotor_index >= 0 && rotor_index < rotor_count)
		rotating_movements_[rotor_index]->RotationRate.Yaw = radsPerSec * 180.0f / M_PIf * RotatorFactor;
}

msr::airlib::MavLinkDroneController::ConnectionInfo AFlyingPawn::getMavConnectionInfo()
{
	msr::airlib::MavLinkDroneController::ConnectionInfo connection_info;
	connection_info.vehicle_name = std::string(TCHAR_TO_UTF8(*VehicleName));
	connection_info.use_serial = UseSerial;
	connection_info.ip_address = std::string(TCHAR_TO_UTF8(*UdpIP));
	connection_info.ip_port = UdpPort;
	connection_info.serial_port = std::string(TCHAR_TO_UTF8(*SerialPortName));
	connection_info.baud_rate = SerialBaudrate;

	return connection_info;
}

void AFlyingPawn::setupComponentReferences()
{
	fpv_camera_ = Cast<APIPCamera>(
		(UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("LeftPIPCamera")))->GetChildActor());

	for (auto i = 0; i < 4; ++i) {
		rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
	}
}


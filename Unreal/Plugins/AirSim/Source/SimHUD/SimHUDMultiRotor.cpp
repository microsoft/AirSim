#include "SimHUDMultiRotor.h"
#include "AirSim.h"
#include "SimMode/SimModeWorldMultiRotor.h"


ASimHUDMultiRotor::ASimHUDMultiRotor()
{
}


ASimHUDMultiRotor::~ASimHUDMultiRotor()
{
}

void ASimHUDMultiRotor::CreateSimMode()
{
	//create simmode
	FActorSpawnParameters simmode_spawn_params;
	simmode_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
	simmode_ = this->GetWorld()->SpawnActor<ASimModeWorldMultiRotor>(FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);
}

// Fill out your copyright notice in the Description page of Project Settings.

#include "TrafficLib.h"
#include "SplineViolationChecker.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"


// get vehicle manager
ANPCVehicleManager* UTrafficLib::getVehicleManager(UWorld* world)
{
	if (world)
	{
		TArray<AActor*> VehicleManagers;

		UGameplayStatics::GetAllActorsOfClass(world, ANPCVehicleManager::StaticClass(), VehicleManagers);

		if (VehicleManagers.Num() > 1)
		{
			// report error for having multiple vehicle managers
			UE_LOG(LogTemp, Warning, TEXT("Warning, TrafficLib found more than one vehicle manager!"));
		}
		if (VehicleManagers.IsValidIndex(0) && VehicleManagers[0])
		{
			ANPCVehicleManager* CastedVehicleManager = Cast<ANPCVehicleManager>(VehicleManagers[0]);
			if (CastedVehicleManager)
			{
				return CastedVehicleManager;
			}
			
		}
		UE_LOG(LogTemp, Warning, TEXT("Warning, TrafficLib found no valid vehicle managers!"));
		return NULL;
	}

	return NULL;
}

void UTrafficLib::setVehicleDensity(UWorld* world, int32 new_density)
{
	ANPCVehicleManager* VehicleManager = getVehicleManager(world);
	if (VehicleManager)
	{
		VehicleManager->initSpawnCarsOnPaths(new_density);
	}

}
int32 UTrafficLib::getVehicleDensity(UWorld* world)
{
	ANPCVehicleManager* VehicleManager = getVehicleManager(world);
	if (VehicleManager)
	{
		return VehicleManager->getCurrentNumCarsPerPath();
	}

	return -1;
}
void UTrafficLib::setPedDensity(UWorld* world, int32 new_density)
{
	ANPCPedestrianManager* PedManager = getPedManager(world);
	if (PedManager)
	{
		PedManager->spawnPedestrians(new_density);
	}
}
int32 UTrafficLib::getPedDensity(UWorld* world)
{
	ANPCPedestrianManager* PedManager = getPedManager(world);
	if (PedManager)
	{
		return PedManager->getCurrentNumPedsPerSpline();
	}
	return 0;
}
// get ped manager
ANPCPedestrianManager* UTrafficLib::getPedManager(UWorld* world)
{
	if (world)
	{
		TArray<AActor*> PedManagers;

		UGameplayStatics::GetAllActorsOfClass(world, ANPCPedestrianManager::StaticClass(), PedManagers);

		if (PedManagers.Num() > 1)
		{
			// report error for having multiple vehicle managers
			UE_LOG(LogTemp, Warning, TEXT("Warning, TrafficLib found more than one ped manager!"));
		}
		if (PedManagers.IsValidIndex(0) && PedManagers[0])
		{
			ANPCPedestrianManager* CastedPedManager = Cast<ANPCPedestrianManager>(PedManagers[0]);
			if (CastedPedManager)
			{
				return CastedPedManager;
			}

		}
		UE_LOG(LogTemp, Warning, TEXT("Warning, TrafficLib found no valid ped managers!"));
		return NULL;
	}

	return NULL;
}
FString UTrafficLib::getTimeOnly(FString Time)
{
	FString NewTime = Time.Replace(TEXT("-"), TEXT(""), ESearchCase::IgnoreCase);
	NewTime.RemoveAt(0, 8, true);

	return NewTime;
	/*for (int32 i = 0; i < 8; i++)
	{
		Time.RemoveAt(0, 8);
	}*/
	//float 
	//return FCString::Atof(*Time);
}
void UTrafficLib::setLocalPawns(UWorld* world, TArray<AActor*> local_pawns)
{
	ANPCPedestrianManager* PedManager = getPedManager(world);
	if (PedManager)
	{
		PedManager->local_pawns_ = local_pawns;
	}
	ANPCVehicleManager* VehicleManager = getVehicleManager(world);
	if (VehicleManager)
	{
		VehicleManager->local_pawns_ = local_pawns;
	}
}
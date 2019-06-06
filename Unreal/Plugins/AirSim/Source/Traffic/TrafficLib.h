// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "NPCPedestrianManager.h"
#include "NPCVehicleManager.h"
#include "TrafficLib.generated.h"

//class ASplineViolationChecker;

/**
 * 
 */
UCLASS()
class AIRSIM_API UTrafficLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	//TSubClassOf<ASplineViolationChecker> violation_checker_class = ASplineViolationChecker::StaticClass();

	// get vehicle manager
	static ANPCVehicleManager* getVehicleManager(UWorld* World);

	// get ped manager
	static ANPCPedestrianManager* getPedManager(UWorld* World);

	// max vehicle and ped density. the UI scales to this value and does not permit higher values
	UFUNCTION(BlueprintCallable, Category = Traffic)
	static int32 getMaxVehicleDensity()
	{
		return 10;
	}
	UFUNCTION(BlueprintCallable, Category = Traffic)
	static int32 getMaxPedDensity()
	{
		return 25;
	}

	// get vehicle density
	UFUNCTION(BlueprintCallable, Category = Traffic)
	static int32 getVehicleDensity(UWorld* World);

	// set vehicle density from 0 -10 (num cars on paths)
	// note, this also forces all vehicles to respawn
	UFUNCTION(BlueprintCallable, Category=Traffic)
	static void setVehicleDensity(UWorld* World, int32 NewDensity);
	// get total num cars

	// get ped density
	UFUNCTION(BlueprintCallable, Category = Traffic)
	static int32 getPedDensity(UWorld* World);

	// set ped density (0-25)
	UFUNCTION(BlueprintCallable, Category = Traffic)
	static void setPedDensity(UWorld* World, int32 NewDensity);
	// get total num peds

	// get just the time, not date
	UFUNCTION(BlueprintCallable, Category = Time)
	static FString getTimeOnly(FString Time);

public:

	// local pawns that are spawned. for performance purposes, pedestrians and vehicles
	// only simulate physics and animate with full detail near the local pawns.
	static void setLocalPawns(UWorld* World, TArray<AActor*> LocalPawns);
};

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SphereComponent.h"
#include "BaseCarPathCollSphereComponent.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API UBaseCarPathCollSphereComponent : public USphereComponent
{
	GENERATED_UCLASS_BODY()
	
public:

	// set collision enabled - enable collision for a bit, then disable it after some time
	// if already enabled, just extend the timer

	void enableCollision(float auto_disable_delay);

	void disableCollision();

	// for comparing if our path index is the same, if we do detect a collision
	UPROPERTY()
	int32 PathIndex;

	// when its collision was last enabled, when clolision checking compare how old/new, older one has right of way (started moving first)
	UPROPERTY()
	float enabled_time;

	UPROPERTY()
	float DisableTime;

	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

	UPROPERTY()
	bool bDrawDebug;
	
};

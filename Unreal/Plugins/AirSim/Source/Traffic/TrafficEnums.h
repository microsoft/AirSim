// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

UENUM(BlueprintType)		//"BlueprintType" is essential to include
enum class ETrafficStatus : uint8
{
        TRAFFIC_GREEN 	UMETA(DisplayName="Green"),
        TRAFFIC_YELLOW 	UMETA(DisplayName="Yellow"),
		TRAFFIC_RED	UMETA(DisplayName="Red")
};

/**
 * 
 */
class AIRSIM_API TrafficEnums
{
public:
	TrafficEnums();
	~TrafficEnums();
};

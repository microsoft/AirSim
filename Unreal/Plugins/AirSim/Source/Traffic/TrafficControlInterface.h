// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "TrafficEnums.h"
#include "TrafficControlInterface.generated.h"

// This class does not need to be modified.
UINTERFACE(MinimalAPI, BlueprintType)
class UTrafficControlInterface : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class AIRSIM_API ITrafficControlInterface
{
	GENERATED_BODY()

public:
	// NOTE: For interface implementation, see https://wiki.unrealengine.com/Interfaces_in_C++
	// new interface standards allow for c++ and blueprint implementation
	// c++ implementation requires overriding function_Implementation(), but can be overriden by blueprints

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category="Traffic")
    void TrafficGreenLight();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category="Traffic")
    void TrafficYellowLight();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category="Traffic")
    void TrafficRedLight();

    /** make this get whatever our current traffic light variable is */
    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category="Traffic")
	ETrafficStatus GetTrafficLightStatus() const;
};

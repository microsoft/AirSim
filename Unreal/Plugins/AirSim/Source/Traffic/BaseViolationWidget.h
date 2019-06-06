// Fill out your copyright notice in the Description page of Project Settings.
// made a base c++ class so we can set the violation checker actor here, which the widget uses to get violation values

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "BaseViolationWidget.generated.h"

class ASplineViolationChecker;
/**
 * 
 */
UCLASS()
class AIRSIM_API UBaseViolationWidget : public UUserWidget
{
	GENERATED_BODY()
	
public:

	// the violation checker that has our violations
	UPROPERTY(BlueprintReadOnly, Category=AirSim)
	ASplineViolationChecker* violation_checker;
	
	
};

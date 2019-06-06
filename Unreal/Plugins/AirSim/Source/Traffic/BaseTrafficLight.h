// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficControlInterface.h"
#include "BaseTrafficLight.generated.h"

UCLASS()
class AIRSIM_API ABaseTrafficLight : public AActor, public ITrafficControlInterface
{
	GENERATED_BODY()
	
public:	
	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		void TrafficGreenLight();

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		void TrafficYellowLight();

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		void TrafficRedLight();

	/** make this get whatever our current traffic light variable is */
	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		ETrafficStatus GetTrafficLightStatus() const;


	virtual void TrafficGreenLight_Implementation() override
	{
		next_traffic_status_ = ETrafficStatus::TRAFFIC_GREEN;
	}
	virtual void TrafficYellowLight_Implementation() override
	{
		next_traffic_status_ = ETrafficStatus::TRAFFIC_YELLOW;
	}
	virtual void TrafficRedLight_Implementation() override
	{
		next_traffic_status_ = ETrafficStatus::TRAFFIC_RED;
	}
	/** make this get whatever our current traffic light variable is */
	virtual ETrafficStatus GetTrafficLightStatus_Implementation() const override
	{
		return next_traffic_status_;
	}



	// Sets default values for this actor's properties
	ABaseTrafficLight();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	ETrafficStatus next_traffic_status_;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};

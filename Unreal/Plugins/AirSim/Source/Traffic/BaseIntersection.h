// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BaseCarPath.h"
#include "BaseTrafficLight.h"
//#include "BaseTrafficGroupControlVolume.h"
//#include "BasePedestrianControlVolume.h"
//#include "PedestrianCrossingPath.h"
#include "TrafficControlInterface.h"
#include "GameFramework/Actor.h"
#include "Components/TextRenderComponent.h"
#include "BaseIntersection.generated.h"

class UStaticMeshComponent;

USTRUCT(BlueprintType)
struct FTrafficGroup
{
	GENERATED_USTRUCT_BODY()
	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	//TArray <class ABaseCarPath*> CarPathArray;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	TArray <class ABaseTrafficLight*> TrafficLights;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	//TArray <class ABaseTrafficGroupControlVolume*> TrafficVolumes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	TArray <class ABaseCarPath*> CarPaths;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=TrafficGroup)
	TArray <class ABasePedestrianPath*> PedestrianCrossingPaths;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=TrafficGroup)
	//TArray <class APedestrianCrossingPath*> PedestrianCrossingPaths;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	//TArray <class ABasePedestrianControlVolume*> PedestrianVolumes;

	// duration in seconds before we switch to next traffic group
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	float Duration = 30.0f;

	// duration before changing pedestrian light
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	float PedestrianDuration = 15.0f;
};

UCLASS()
class AIRSIM_API ABaseIntersection : public AActor, public ITrafficControlInterface
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
	ABaseIntersection();

	virtual void OnConstruction(const FTransform& Transform) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=Collision)
	class UStaticMeshComponent* CollMesh;



protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// debug text that shows on end of paths connecting into this intersection
	UPROPERTY(EditAnywhere, Category=Debug)
	float debug_text_scale;

	/** STRUCT to store our inventory for save games */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TrafficGroup)
	TArray<FTrafficGroup> TrafficGroupArray;

	//UFUNCTION()
	//void cycleTrafficGroups();

	// this is from the traffic group's duration value
	UPROPERTY()
	float next_traffic_change_time_;

	UPROPERTY()
	float yellow_light_duration_;

	// time before the current traffic group goes green. make sure this is greater than yellow light duration!
	UPROPERTY()
	float start_current_traffic_group_delay_;

	UPROPERTY(VisibleAnywhere)
	int32 current_traffic_group_index_;


	//void currentTrafficGroupYellowLight();

	//void nextTrafficGroup();

	//void startCurrentTrafficGroupDelayed(float delay);

	void startCurrentTrafficGroup();

	void lastTrafficGroupRedLightDelayed(float delay);

	void lastTrafficGroupRedLight();

	void setAllTrafficGroupStatus(ETrafficStatus new_traffic_status, int32 index);

	FTimerHandle yellowLight_Timer_;

	FTimerHandle currentTrafficGroup_Timer_;

	UPROPERTY()
	ETrafficStatus next_traffic_status_;

	UFUNCTION(BlueprintCallable, CallInEditor, Category=Init)
	void initAllIntersections();

	void setTrafficGroupIntersections();

	UPROPERTY(VisibleAnywhere, Category=Debug)
	TArray<UTextRenderComponent*> text_comps_;

	UFUNCTION(BlueprintCallable, CallInEditor, Category = Init)
	void destroyTextComps();

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};

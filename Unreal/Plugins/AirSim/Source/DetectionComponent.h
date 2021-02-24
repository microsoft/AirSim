// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "ObjectFilter.h"
#include "DetectionComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class AIRSIM_API UDetectionComponent : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UDetectionComponent();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	TMap<AActor*, FBox2D> GetDetections() const;
private:
	bool CalcBoundingFromViewInfo(AActor* Actor, FBox2D& BoxOut);

public:
	UPROPERTY()
		FObjectFilter ObjectFilter;

	UPROPERTY(EditAnywhere, Category = "Tracked Actors")
		float MaxDistanceToCamera;

	UPROPERTY()
		UTextureRenderTarget2D* TextureTarget;
private:

	UPROPERTY()
		USceneCaptureComponent2D* SceneCaptureComponent2D;

	UPROPERTY()
		TMap<AActor*, FBox2D> CachedBoundingBoxes;
};

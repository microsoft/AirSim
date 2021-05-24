// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "ObjectFilter.h"
#include "DetectionComponent.generated.h"

USTRUCT()
struct FDetectionInfo
{
    GENERATED_BODY()

    UPROPERTY()
    AActor* Actor;

    UPROPERTY()
    FBox2D Box2D;

    UPROPERTY()
    FBox Box3D;

    UPROPERTY()
    FTransform RelativeTransform;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
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

    TArray<FDetectionInfo> GetDetections() const;

private:
    bool CalcBoundingFromViewInfo(AActor* Actor, FBox2D& BoxOut);

    FVector GetRelativeLocation(FVector InLocation);

    FRotator GetRelativeRotation(FVector InLocation, FRotator InRotation);

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
    TArray<FDetectionInfo> CachedDetections;
};

// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "VehiclePawnWrapper.h"
#include "WheeledVehicle.h"
#include "Car4x4Pawn.generated.h"

class UPhysicalMaterial;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class UInputComponent;
class UAudioComponent;

UCLASS(config = Game)
class ACar4x4Pawn : public AWheeledVehicle
{
    GENERATED_BODY()

    /** Spring arm that will offset the camera */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    USpringArmComponent* SpringArm;

    /** Camera component that will be our viewpoint */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UCameraComponent* Camera;

    /** SCene component for the In-Car view origin */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USceneComponent* InternalCameraBase;

    /** Camera component for the In-Car view */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UCameraComponent* InternalCamera;

    /** Text component for the In-Car speed */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UTextRenderComponent* InCarSpeed;

    /** Text component for the In-Car gear */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UTextRenderComponent* InCarGear;

    /** Audio component for the engine sound */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UAudioComponent* EngineSoundComponent;

public:
    ACar4x4Pawn();

    /** The current speed as a string eg 10 km/h */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    FText SpeedDisplayString;

    /** The current gear as a string (R,N, 1,2 etc) */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    FText GearDisplayString;

    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    /** The color of the incar gear text in forward gears */
    FColor	GearDisplayColor;

    /** The color of the incar gear text when in reverse */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    FColor	GearDisplayReverseColor;

    /** Are we in reverse gear */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly)
    bool bInReverseGear;

    /** Initial offset of incar camera */
    FVector InternalCameraOrigin;

    void setupInputBindings();

    // Begin Actor interface
    virtual void Tick(float Delta) override;

    VehiclePawnWrapper* getVehiclePawnWrapper();
    void initializeForBeginPlay();

    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;
protected:
    virtual void BeginPlay() override;

public:
    // End Actor interface

    /** Handle pressing forwards */
    void MoveForward(float Val);

    /** Setup the strings used on the hud */
    void UpdateInCarHUD();

    /** Update the physics material used by the vehicle mesh */
    void UpdatePhysicsMaterial();

    /** Handle pressing right */
    void MoveRight(float Val);
    /** Handle handbrake pressed */
    void OnHandbrakePressed();
    /** Handle handbrake released */
    void OnHandbrakeReleased();

    static const FName LookUpBinding;
    static const FName LookRightBinding;
    static const FName EngineAudioRPM;

private:
    /** Update the gear and speed strings */
    void UpdateHUDStrings();

    /* Are we on a 'slippery' surface */
    bool bIsLowFriction;
    /** Slippery Material instance */
    UPhysicalMaterial* SlipperyMaterial;
    /** Non Slippery Material instance */
    UPhysicalMaterial* NonSlipperyMaterial;

    std::unique_ptr<VehiclePawnWrapper> wrapper_;


public:
    /** Returns SpringArm subobject **/
    FORCEINLINE USpringArmComponent* GetSpringArm() const { return SpringArm; }
    /** Returns Camera subobject **/
    FORCEINLINE UCameraComponent* GetCamera() const { return Camera; }
    /** Returns InternalCamera subobject **/
    FORCEINLINE UCameraComponent* GetInternalCamera() const { return InternalCamera; }
    /** Returns InCarSpeed subobject **/
    FORCEINLINE UTextRenderComponent* GetInCarSpeed() const { return InCarSpeed; }
    /** Returns InCarGear subobject **/
    FORCEINLINE UTextRenderComponent* GetInCarGear() const { return InCarGear; }
    /** Returns EngineSoundComponent subobject **/
    FORCEINLINE UAudioComponent* GetEngineSoundComponent() const { return EngineSoundComponent; }
};

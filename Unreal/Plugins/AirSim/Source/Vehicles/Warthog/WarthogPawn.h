// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "vehicles/warthog/api/WarthogApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "PawnEvents.h"
#include "PIPCamera.h"
#include "WarthogPawn.generated.h"

class UCameraComponent;
UCLASS(config = Game)
class AWarthogPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AWarthogPawn();
    virtual void BeginPlay() override;
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                           FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface
    void initializeForBeginPlay(bool engine_sound);
    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
    PawnEvents* getPawnEvents()
    {
        return &pawn_events_;
    }
    const msr::airlib::WarthogApiBase::WarthogControls& getKeyBoardControls() const
    {
        return keyboard_controls_;
    }


public:	
	// Called every frame

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
    UFUNCTION(BlueprintCallable, meta = (Tooltip = "Return our PawnMovementComponent, if we have one."), Category = Pawn)
    float GetLinearVelocity();
    UFUNCTION(BlueprintCallable, meta = (Tooltip = "Return our PawnMovementComponent, if we have one."), Category = Pawn)
    float GetAngularVelocity();
    void SetLinearVelocity(float);
    void SetAngularVelocity(float);

private:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    UClass* pip_camera_class_;

    PawnEvents pawn_events_;
    UPROPERTY()
    USceneComponent* camera_front_center_;
    USceneComponent* camera_rig_;
    msr::airlib::WarthogApiBase::WarthogControls keyboard_controls_;
    float desired_liner_vel_ ;
    float desired_angular_vel_;

};

// Fill out your copyright notice in the Description page of Project Settings.


#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "vehicles/warthog/api/WarthogApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "WarthogPawn.generated.h"

UCLASS(config = Game)
class AWarthogPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AWarthogPawn();
    const msr::airlib::WarthogApiBase::WarthogControls& getKeyBoardControls() const
    {
        return keyboard_controls_;
    }


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
    UFUNCTION(BlueprintCallable, meta = (Tooltip = "Return our PawnMovementComponent, if we have one."), Category = Pawn)
    float GetLinearVelocity();
    UFUNCTION(BlueprintCallable, meta = (Tooltip = "Return our PawnMovementComponent, if we have one."), Category = Pawn)
    float GetAngularVelocity();
    void SetLinearVelocity(float);
    void SetAngularVelocity(float);

private:
    msr::airlib::WarthogApiBase::WarthogControls keyboard_controls_;
    float desired_liner_vel_ ;
    float desired_angular_vel_;

};

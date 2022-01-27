// Fill out your copyright notice in the Description page of Project Settings.


#include "Vehicles/Warthog/WarthogPawn.h"
// Sets default values
AWarthogPawn::AWarthogPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AWarthogPawn::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AWarthogPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void AWarthogPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

float AWarthogPawn::GetLinearVelocity()
{
    return desired_liner_vel_;
}

float AWarthogPawn::GetAngularVelocity()
{
    return desired_angular_vel_;
}

void AWarthogPawn::SetLinearVelocity(float linear_vel)
{
    desired_liner_vel_ = linear_vel;
}

void AWarthogPawn::SetAngularVelocity(float angular_vel)
{
    desired_angular_vel_ = angular_vel;
}

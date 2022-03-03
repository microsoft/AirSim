// Fill out your copyright notice in the Description page of Project Settings.


#include "Vehicles/Warthog/WarthogPawn.h"
#include "AirBlueprintLib.h"
// Sets default values
AWarthogPawn::AWarthogPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
 //   static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
 //   pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;
 //   camera_rig_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_rig_")); 
 //   camera_front_center_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_"));
 //   camera_front_center_->SetRelativeLocation(FVector(200, 0, 100)); //center
 //   camera_front_center_->SetupAttachment(camera_rig_);
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
void AWarthogPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                         FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
}
void AWarthogPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    /*camera_front_center_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_right_ = nullptr;
    camera_driver_ = nullptr;
    camera_back_center_ = nullptr;

    camera_front_center_base_ = nullptr;
    camera_front_left_base_ = nullptr;
    camera_front_right_base_ = nullptr;
    camera_driver_base_ = nullptr;
    camera_back_center_base_ = nullptr;*/
    return;
}
void AWarthogPawn::initializeForBeginPlay(bool engine_sound)
{
    return;
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> AWarthogPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    /* cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("fpv", camera_driver_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_driver_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("", camera_front_center_);*/

    return cameras;
}

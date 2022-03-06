// Fill out your copyright notice in the Description page of Project Settings.


#include "Vehicles/Warthog/WarthogPawn.h"
#include "AirBlueprintLib.h"
// Sets default values
AWarthogPawn::AWarthogPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    kp_ = 0;
    kd_ = 0;
    ki_ = 0;
    prev_l_error_ = 0;
    prev_r_error_ = 0;
    left_error_sum_ = 0;
    right_error_sum_ = 0;
    warthog_half_diff_radius_ = 0.285;
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
    pawn_events_.getPawnTickSignal().emit(DeltaTime);
    DoPidUpdate(DeltaTime);
}

// Called to bind functionality to input
void AWarthogPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

float AWarthogPawn::GetLinearVelocity()
{
    return desired_linear_vel_;
}

float AWarthogPawn::GetAngularVelocity()
{
    return desired_angular_vel_;
}

void AWarthogPawn::SetDesiredVelocities(float linear_vel, float angular_vel)
{
    desired_linear_vel_ = linear_vel;
    desired_angular_vel_ = angular_vel;
}
void AWarthogPawn::GetWarthogMesh(UPrimitiveComponent* temp) 
{
    war_mesh_ = temp;
    return;
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
void AWarthogPawn::SetKp(float kp)
{
    kp_ = kp;
    return;
}

void AWarthogPawn::SetKd(float kd)
{
    kd_ = kd;
    return;
}
void AWarthogPawn::SetKi(float ki)
{
    ki_ = ki;
    return;
}
void AWarthogPawn::GetCurrentVOmega(float&v, float&omega)
{
    FMatrix t = war_mesh_->GetRelativeTransform().ToMatrixWithScale();
    FVector lin_vel_vec = war_mesh_->GetComponentVelocity();
    FVector lin_vel_vec_local_frame = t.InverseTransformVector(lin_vel_vec);
    v = lin_vel_vec_local_frame.X/100.0; //report in m/s
    omega = -1 * war_mesh_->GetPhysicsAngularVelocityInRadians().Z;
   // GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("lin_vel_cpp %f"), v));
   // GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("ang_vel_cpp %f"), omega));
}
void AWarthogPawn::SetWheelVelocities(float v, float omega, float& vl, float& vr)
{
    vl = v - omega * warthog_half_diff_radius_;
    vr = v + omega * warthog_half_diff_radius_;
}
void AWarthogPawn::DoPidUpdate(float dt)
{
    float v, omega, vl, vr, vl_desired, vr_desired;
    float left_torque;
    float right_torque;
    GetCurrentVOmega(v, omega);
    SetWheelVelocities(v, omega, vl, vr);
    SetWheelVelocities(desired_linear_vel_, desired_angular_vel_, vl_desired, vr_desired);
    float curr_l_error = vl_desired - vl;
    float curr_r_error = vr_desired - vr;
    left_torque = kp_ * (curr_l_error) + kd_ * (curr_l_error - prev_l_error_) + (ki_ * (left_error_sum_ + curr_l_error ));
    right_torque = kp_ * (curr_r_error) + kd_ * (curr_r_error - prev_r_error_)  + (ki_ * (right_error_sum_ + curr_r_error));
    left_error_sum_ = left_error_sum_ + curr_l_error ;
    right_error_sum_ = right_error_sum_ + curr_r_error ;
    prev_l_error_ = curr_l_error;
    prev_r_error_ = curr_r_error;
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("left_t %f"), left_torque));
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("right_t_cpp %f"), right_torque));


}


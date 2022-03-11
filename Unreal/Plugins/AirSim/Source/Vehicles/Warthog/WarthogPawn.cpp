// Fill out your copyright notice in the Description page of Project Settings.


#include "Vehicles/Warthog/WarthogPawn.h"
#include "AirBlueprintLib.h"
// Sets default values
AWarthogPawn::AWarthogPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;
    kp_ = 0;
    kd_ = 0;
    ki_ = 0;
    prev_l_error_ = 0;
    prev_r_error_ = 0;
    left_error_sum_ = 0;
    right_error_sum_ = 0;
    warthog_half_diff_radius_ = 0.568;
    left_torque_ = 0;
    right_torque_ = 0;
    pid_update_time_ = 0.02;
    time_since_last_pid_ = 0;
    curr_v = 0;
    curr_w = 0;
 //   static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
 //   pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;
 //   camera_rig_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_rig_")); 
 //   camera_front_center_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_"));
 //   camera_front_center_->SetRelativeLocation(FVector(200, 0, 100)); //center
 //   camera_front_center_->SetupAttachment(camera_rig_);
}

// Called when the game starts or when spawned
void AWarthogPawn::SetPidUpdateTime(float pid_update_time)
{
    pid_update_time_ = pid_update_time;
}
void AWarthogPawn::BeginPlay()
    {
	Super::BeginPlay();
	
}

// Called every frame
void AWarthogPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    float v, omega;
    GetCurrentVOmega(v, omega);
    //curr_v = v;
    //curr_w = omega;
    pawn_events_.getPawnTickSignal().emit(DeltaTime);
    time_since_last_pid_ = time_since_last_pid_ + DeltaTime;
    DoLQRUpdate(DeltaTime);
   // DoPidUpdate(time_since_last_pid_);
    if (time_since_last_pid_ >= pid_update_time_) {
       // DoPidUpdate(time_since_last_pid_);
        time_since_last_pid_ = 0;
    }
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("lin_vel_cpp %f"), v));
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("ang_vel_cpp %f"), omega));
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("step time %f"), DeltaTime));
}
float AWarthogPawn::GetLeftTorque()
{
    return left_torque_;
}
float AWarthogPawn::GetRightTorque()
{
    return right_torque_;
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
    //camera_front_center_base_ = this->CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_base_"));
    camera_front_center_base_ = NewObject<USceneComponent>(this, TEXT("camera_front_center_base_"));
    camera_front_center_base_->RegisterComponent();
    camera_front_center_base_->SetRelativeLocation(FVector(-700, 0, 100)); //center
    FAttachmentTransformRules rig_att_rules(EAttachmentRule::KeepRelative, true);
    camera_front_center_base_->AttachToComponent(war_mesh_, rig_att_rules);
    FTransform camera_transform(FVector::ZeroVector);
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_center"));
    camera_front_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_center_->AttachToComponent(camera_front_center_base_, FAttachmentTransformRules::KeepRelativeTransform);
    return;
}
void AWarthogPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                         FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
}
void AWarthogPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_center_ = nullptr;
    camera_front_center_base_ = nullptr;
    /* camera_front_left_ = nullptr;
    camera_front_right_ = nullptr;
    camera_driver_ = nullptr;
    camera_back_center_ = nullptr;

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
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("0", camera_front_center_);
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
void AWarthogPawn::SetLQRParams(float lin, float ang)
{
    lqr_param_lin_ = lin;
    lqr_param_ang_ = ang;
    return;
}
void AWarthogPawn::GetCurrentVOmega(float&v, float&omega)
{
    //return;
    FMatrix t = war_mesh_->GetRelativeTransform().ToMatrixWithScale();
    FVector lin_vel_vec = war_mesh_->GetComponentVelocity();
    FVector lin_vel_vec_local_frame = t.InverseTransformVector(lin_vel_vec);
    v = lin_vel_vec_local_frame.X/100.0; //report in m/s
    omega = -1 * war_mesh_->GetPhysicsAngularVelocityInRadians().Z;
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("lin_vel_cpp %f"), v));
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("ang_vel_cpp %f"), omega));
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("diff_rad %f"), warthog_half_diff_radius_));
}
void AWarthogPawn::SetWheelVelocities(float v, float omega, float& vl, float& vr)
{
    vl = v - (omega * warthog_half_diff_radius_);
    vr = v + (omega * warthog_half_diff_radius_);
}
void AWarthogPawn::DoPidUpdate(float dt)
{
    float v, omega, vl, vr, vl_desired, vr_desired;
    GetCurrentVOmega(v, omega);
    SetWheelVelocities(v, omega, vl, vr);
    SetWheelVelocities(desired_linear_vel_, desired_angular_vel_, vl_desired, vr_desired);
    //if ((FMath::Abs(vl_desired) <= 0.2) && (FMath::Abs(vr_desired) <= 0.2)) {
      //  left_torque_ = 0; 
      //  right_torque_ = 0; 
       // return;
   // }
    float curr_l_error = vl_desired - vl;
    float curr_r_error = vr_desired - vr;
    left_torque_ = kp_ * (curr_l_error) + 
        kd_ * (curr_l_error - prev_l_error_)/dt + 
        (ki_ * (left_error_sum_ + curr_l_error*dt ));
    right_torque_ = kp_ * (curr_r_error) + 
        kd_ * (curr_r_error - prev_r_error_)/dt + 
        (ki_ * (right_error_sum_ + curr_r_error*dt));
    left_error_sum_ = left_error_sum_ + curr_l_error*dt ;
    right_error_sum_ = right_error_sum_ + curr_r_error*dt ;
    prev_l_error_ = curr_l_error;
    prev_r_error_ = curr_r_error;
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("left_t %f"), left_torque_));
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("right_t_cpp %f"), right_torque_));
    GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Green, FString::Printf(TEXT("right_t_cppkk %f"), ki_));
    curr_v = curr_l_error;
    curr_w = curr_r_error;
    float v_err = v - desired_linear_vel_;
    float w_err = omega - desired_angular_vel_;
    right_torque_ = -(kp_ * v_err + kd_ * w_err)/2;
    left_torque_ = -(kp_ * v_err - kd_ * w_err)/2;
    return;
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("left_t %f"), curr_l_error));
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("right_t_cpp %f"), curr_r_error));


}
void AWarthogPawn::DoLQRUpdate(float dt)
{
    float v, omega;
    GetCurrentVOmega(v, omega);
    float v_err = v - desired_linear_vel_;
    float w_err = omega - desired_angular_vel_;
    right_torque_ = -(lqr_param_lin_ * v_err + lqr_param_ang_ * w_err) / 2;
    left_torque_ = -(lqr_param_lin_ * v_err - lqr_param_ang_* w_err) / 2;
    return;
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("left_t %f"), curr_l_error));
    //GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Yellow, FString::Printf(TEXT("right_t_cpp %f"), curr_r_error));
}


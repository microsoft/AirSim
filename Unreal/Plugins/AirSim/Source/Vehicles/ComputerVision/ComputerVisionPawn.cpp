#include "ComputerVisionPawn.h"
#include "Engine/World.h"
#include "ManualPoseController.h"

AComputerVisionPawn::AComputerVisionPawn()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

    camera_front_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_base_"));
    camera_front_center_base_->SetRelativeLocation(FVector(0, 0, 0)); //center
    camera_front_left_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_left_base_"));
    camera_front_left_base_->SetRelativeLocation(FVector(0, -12.5, 0)); //left
    camera_front_right_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_right_base_"));
    camera_front_right_base_->SetRelativeLocation(FVector(0, 12.5, 0)); //right
    camera_bottom_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_bottom_center_base_"));
    camera_bottom_center_base_->SetRelativeLocation(FVector(0, 0, 0)); //right
    camera_back_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_back_center_base_"));
    camera_back_center_base_->SetRelativeLocation(FVector(0, 0, 0)); //right

    camera_front_center_base_->SetupAttachment(RootComponent);
    camera_front_left_base_->SetupAttachment(RootComponent);
    camera_front_right_base_->SetupAttachment(RootComponent);
    camera_bottom_center_base_->SetupAttachment(RootComponent);
    camera_back_center_base_->SetupAttachment(RootComponent);
}

void AComputerVisionPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                                    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
}

void AComputerVisionPawn::initializeForBeginPlay()
{
    //put camera little bit above vehicle
    FTransform camera_transform(FVector::ZeroVector);
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_center"));
    camera_front_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_center_->AttachToComponent(camera_front_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_left"));
    camera_front_left_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_left_->AttachToComponent(camera_front_left_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_right"));
    camera_front_right_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_right_->AttachToComponent(camera_front_right_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_bottom_center"));
    camera_bottom_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_,
                                                                     FTransform(FRotator(-90, 0, 0), FVector::ZeroVector),
                                                                     camera_spawn_params);
    camera_bottom_center_->AttachToComponent(camera_bottom_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_back_center"));
    camera_back_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_,
                                                                   FTransform(FRotator(0, -180, 0), FVector::ZeroVector),
                                                                   camera_spawn_params);
    camera_back_center_->AttachToComponent(camera_back_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    manual_pose_controller_ = NewObject<UManualPoseController>(this, "ComputerVision_ManualPoseController");
    manual_pose_controller_->initializeForPlay();
    manual_pose_controller_->setActor(this);
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> AComputerVisionPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("bottom_center", camera_bottom_center_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_bottom_center_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("fpv", camera_front_center_);
    cameras.insert_or_assign("", camera_front_center_);

    return cameras;
}

void AComputerVisionPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_center_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_right_ = nullptr;
    camera_bottom_center_ = nullptr;
    camera_back_center_ = nullptr;

    camera_front_center_base_ = nullptr;
    camera_front_left_base_ = nullptr;
    camera_front_right_base_ = nullptr;
    camera_bottom_center_base_ = nullptr;
    camera_back_center_base_ = nullptr;

    manual_pose_controller_ = nullptr;
}

void AComputerVisionPawn::Tick(float Delta)
{
    Super::Tick(Delta);
    pawn_events_.getPawnTickSignal().emit(Delta);

    //update ground level
    if (manual_pose_controller_->getActor() == this) {
        manual_pose_controller_->updateActorPose(Delta);
    }
}

void AComputerVisionPawn::BeginPlay()
{
    Super::BeginPlay();
}

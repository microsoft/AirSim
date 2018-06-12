#include "ComputerVisionPawn.h"
#include "Engine/World.h"


AComputerVisionPawn::AComputerVisionPawn()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

    // Create In-Car camera component 
    camera_front_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_base_"));
    camera_front_center_base_->SetRelativeLocation(FVector(200, 0, 100)); //center
    camera_front_center_base_->SetupAttachment(GetRootComponent());
    camera_front_left_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_left_base_"));
    camera_front_left_base_->SetRelativeLocation(FVector(200, -100, 100)); //left
    camera_front_left_base_->SetupAttachment(GetRootComponent());
    camera_front_right_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_right_base_"));
    camera_front_right_base_->SetRelativeLocation(FVector(200, 100, 100)); //right
    camera_front_right_base_->SetupAttachment(GetRootComponent());

    manual_pose_controller_ = NewObject<UManualPoseController>();
}

void AComputerVisionPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

void AComputerVisionPawn::initializeForBeginPlay()
{
    //put camera little bit above vehicle
    FTransform camera_transform(FVector::ZeroVector);
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    camera_spawn_params.Name = "camera_front_center";
    camera_front_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_center_->AttachToComponent(camera_front_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = "camera_front_left";
    camera_front_left_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_left_->AttachToComponent(camera_front_left_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = "camera_front_right";
    camera_front_right_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_right_->AttachToComponent(camera_front_right_base_, FAttachmentTransformRules::KeepRelativeTransform);

    setupInputBindings();

    manual_pose_controller_->initializeForPlay();
    manual_pose_controller_->setActor(this);
}

common_utils::UniqueValueMap<std::string, APIPCamera*> AComputerVisionPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);

    cameras.insert_or_assign("fpv", camera_front_center_);
    cameras.insert_or_assign("", camera_front_center_);

    return cameras;
}

void AComputerVisionPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_center_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_right_ = nullptr;

    camera_front_center_base_ = nullptr;
    camera_front_left_base_ = nullptr;
    camera_front_right_base_ = nullptr;

    manual_pose_controller_ = nullptr;
}

void AComputerVisionPawn::Tick(float Delta)
{
    Super::Tick(Delta);
    pawn_events_.getPawnTickSignal().emit(Delta);

    //update ground level
    if (manual_pose_controller_->getActor() == this) {
        FVector delta_position;
        FRotator delta_rotation;

        manual_pose_controller_->updateDeltaPosition(Delta);
        manual_pose_controller_->getDeltaPose(delta_position, delta_rotation);
        manual_pose_controller_->resetDelta();

        SetActorRelativeLocation(delta_position);
        SetActorRelativeRotation(delta_rotation);
    }
}

void AComputerVisionPawn::BeginPlay()
{
    Super::BeginPlay();
}


/******************* Keyboard bindings*******************/
//This method must be in pawn because Unreal doesn't allow key bindings to non UObject pointers
void AComputerVisionPawn::setupInputBindings()
{

}

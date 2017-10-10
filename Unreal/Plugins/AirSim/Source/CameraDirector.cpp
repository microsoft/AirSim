#include "CameraDirector.h"
#include "AirBlueprintLib.h"

ACameraDirector::ACameraDirector()
{
    PrimaryActorTick.bCanEverTick = true;

    // Create a spring arm component for our chase camera
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));
    SpringArm->SetWorldRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->TargetArmLength = 125.0f;
    SpringArm->bEnableCameraLag = false;
    SpringArm->bEnableCameraRotationLag = false;
    SpringArm->bInheritPitch = true;
    SpringArm->bInheritYaw = true;
    SpringArm->bInheritRoll = true;
}

void ACameraDirector::BeginPlay()
{
    Super::BeginPlay();
}

void ACameraDirector::Tick( float DeltaTime )
{
    Super::Tick( DeltaTime );

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
        manual_pose_controller_->updateActorPose();
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
        //do nothing
    }
    else {
        UAirBlueprintLib::FollowActor(external_camera_, follow_actor_, initial_ground_obs_offset_, ext_obs_fixed_z_);
    }
}

ECameraDirectorMode ACameraDirector::getMode()
{
    return mode_;
}

void ACameraDirector::initializeForBeginPlay(ECameraDirectorMode view_mode, VehiclePawnWrapper* vehicle_pawn_wrapper, APIPCamera* external_camera)
{
    manual_pose_controller_ = NewObject<UManualPoseController>();
    manual_pose_controller_->initializeForPlay();

    setupInputBindings();

    mode_ = view_mode;
    setCameras(external_camera, vehicle_pawn_wrapper);
}

void ACameraDirector::setCameras(APIPCamera* external_camera, VehiclePawnWrapper* vehicle_pawn_wrapper)
{
    external_camera_ = external_camera;
    fpv_camera_ = vehicle_pawn_wrapper->getCameraCount() > 0 ? vehicle_pawn_wrapper->getCamera() : nullptr;
    follow_actor_ = vehicle_pawn_wrapper->getPawn();

    camera_start_location_ = this->GetActorLocation();
    camera_start_rotation_ = this->GetActorRotation();
    initial_ground_obs_offset_ = camera_start_location_ - follow_actor_->GetActorLocation();

    manual_pose_controller_->setActor(external_camera_, false);

    //set initial view mode
    switch (mode_) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME: inputEventFlyWithView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV: inputEventFpvView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER: inputEventGroundView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL: inputEventManualView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE: inputEventSpringArmChaseView(); break;
    default:
        throw std::out_of_range("Unknown view mode specified in CameraDirector::initializeForBeginPlay");
    }
}

void ACameraDirector::attachSpringArm(bool attach)
{
    if (attach) {
        if (follow_actor_ && external_camera_->GetRootComponent()->GetAttachParent() != SpringArm) {
            last_parent_ = external_camera_->GetRootComponent()->GetAttachParent();
            external_camera_->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            SpringArm->AttachToComponent(follow_actor_->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
            SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));
        }

        external_camera_->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform);
        external_camera_->SetActorRelativeLocation(FVector(-225.0, 0.0f, 0.0f));
        external_camera_->SetActorRelativeRotation(FRotator(10.0f, 0.0f, 0.0f));
        //external_camera_->bUsePawnControlRotation = false;
    }
    else {
        if (last_parent_ && external_camera_->GetRootComponent()->GetAttachParent() == SpringArm) {
            external_camera_->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            external_camera_->AttachToComponent(last_parent_, FAttachmentTransformRules::KeepWorldTransform);
        }
    }
}

void ACameraDirector::setMode(ECameraDirectorMode mode)
{
    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE &&
        mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE)
    {
        attachSpringArm(false);
    }

    mode_ = mode;

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL)
        manual_pose_controller_->enableBindings(true);
    else if (external_camera_ != nullptr && manual_pose_controller_->getActor() == external_camera_)
        manual_pose_controller_->enableBindings(false);
    //else someone else is bound to manual pose controller, leave it alone

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE)
        attachSpringArm(true);
}

void ACameraDirector::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventFpvView", EKeys::F, this, &ACameraDirector::inputEventFpvView);
    UAirBlueprintLib::BindActionToKey("inputEventFlyWithView", EKeys::B, this, &ACameraDirector::inputEventFlyWithView);
    UAirBlueprintLib::BindActionToKey("inputEventGroundView", EKeys::Backslash, this, &ACameraDirector::inputEventGroundView);
    UAirBlueprintLib::BindActionToKey("inputEventManualView", EKeys::M, this, &ACameraDirector::inputEventManualView);
    UAirBlueprintLib::BindActionToKey("inputEventSpringArmChaseView", EKeys::Slash, this, &ACameraDirector::inputEventSpringArmChaseView);
}


void ACameraDirector::inputEventFpvView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV);
    external_camera_->disableMain();
    if (fpv_camera_)
        fpv_camera_->showToScreen();
}

void ACameraDirector::inputEventSpringArmChaseView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE);
    external_camera_->showToScreen();
    if (fpv_camera_)
        fpv_camera_->disableMain();
}

void ACameraDirector::inputEventGroundView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER);
    external_camera_->showToScreen();
    if (fpv_camera_)
        fpv_camera_->disableMain();
    ext_obs_fixed_z_ = true;
}

void ACameraDirector::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    fpv_camera_ = nullptr;
    external_camera_ = nullptr;
    follow_actor_ = nullptr;
}

APIPCamera* ACameraDirector::getFpvCamera() const
{
    return fpv_camera_;
}

APIPCamera* ACameraDirector::getExternalCamera() const
{
    return external_camera_;
}

void ACameraDirector::inputEventManualView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL);
}

void ACameraDirector::inputEventFlyWithView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME);
    external_camera_->showToScreen();

    if (follow_actor_)
        external_camera_->SetActorLocationAndRotation(
            follow_actor_->GetActorLocation() + initial_ground_obs_offset_, camera_start_rotation_);
    if (fpv_camera_)
        fpv_camera_->disableMain();
    ext_obs_fixed_z_ = false;
}



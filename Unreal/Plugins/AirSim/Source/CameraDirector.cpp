#include "CameraDirector.h"
#include "GameFramework/PlayerController.h"
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
    SpringArm->CameraRotationLagSpeed = 10.0f;
    SpringArm->bInheritPitch = true;
    SpringArm->bInheritYaw = true;
    SpringArm->bInheritRoll = true;
}

void ACameraDirector::BeginPlay()
{
    Super::BeginPlay();
}

void ACameraDirector::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
        manual_pose_controller_->updateActorPose(DeltaTime);
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) {
        //do nothing, spring arm is pulling the camera with it
    }
    else if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY) {
        //do nothing, we have camera turned off
    }
    else { //make camera move in desired way
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

void ACameraDirector::setupCameraFromSettings()
{
    typedef msr::airlib::AirSimSettings AirSimSettings;
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    if (!external_camera_)
        return;

    int image_count = static_cast<int>(Utils::toNumeric(ImageType::Count));
    for (int image_type = -1; image_type < image_count; ++image_type) {
        external_camera_->setImageTypeSettings(image_type, AirSimSettings::singleton().capture_settings[image_type], 
            AirSimSettings::singleton().noise_settings[image_type]);
    }
}

void ACameraDirector::setCameras(APIPCamera* external_camera, VehiclePawnWrapper* vehicle_pawn_wrapper)
{
    external_camera_ = external_camera;
    follow_actor_ = vehicle_pawn_wrapper->getPawn();
    fpv_camera_ = vehicle_pawn_wrapper->getCameraCount() > fpv_camera_index_ ? vehicle_pawn_wrapper->getCamera(fpv_camera_index_) : nullptr;
    backup_camera_ = backup_camera_index_ >= 0 && vehicle_pawn_wrapper->getCameraCount() > backup_camera_index_ ? vehicle_pawn_wrapper->getCamera(backup_camera_index_) : nullptr;
    camera_start_location_ = external_camera_->GetActorLocation();
    camera_start_rotation_ = external_camera_->GetActorRotation();
    initial_ground_obs_offset_ = camera_start_location_ - follow_actor_->GetActorLocation();

    manual_pose_controller_->setActor(external_camera_, false);

    //set initial view mode
    switch (mode_) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME: inputEventFlyWithView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV: inputEventFpvView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER: inputEventGroundView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL: inputEventManualView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE: inputEventSpringArmChaseView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP: inputEventBackupView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY: inputEventNoDisplayView(); break;
    default:
        throw std::out_of_range("Unsupported view mode specified in CameraDirector::initializeForBeginPlay");
    }

    setupCameraFromSettings();
}

void ACameraDirector::attachSpringArm(bool attach)
{
    if (attach) {
        //If we do have actor to follow AND don't have sprint arm attached to that actor, we will attach it
        if (follow_actor_ && external_camera_->GetRootComponent()->GetAttachParent() != SpringArm) {
            //For car, we want a bit of camera lag, as that is customary of racing video games
            //If the lag is missing, the camera will also occasionally shake.
            //But, lag is not desired when piloting a drone
            SpringArm->bEnableCameraRotationLag = camera_rotation_lag_enabled_;

            //attach spring arm to actor
            SpringArm->AttachToComponent(follow_actor_->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
            SpringArm->SetRelativeLocation(FVector(0.0f, 0.0f, 34.0f));

            //remember current parent for external camera. Later when we remove external
            //camera from spring arm, we will attach it back to its last parent
            last_parent_ = external_camera_->GetRootComponent()->GetAttachParent();
            external_camera_->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            //now attach camera to spring arm
            external_camera_->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepRelativeTransform);
        }

        //For car, we need to move the camera back a little more than for a drone. 
        //Otherwise, the camera will be stuck inside the car
        external_camera_->SetActorRelativeLocation(FVector(follow_distance_, 0.0f, 0.0f));
        external_camera_->SetActorRelativeRotation(FRotator(10.0f, 0.0f, 0.0f));
        //external_camera_->bUsePawnControlRotation = false;
    }
    else { //detach
        if (last_parent_ && external_camera_->GetRootComponent()->GetAttachParent() == SpringArm) {
            external_camera_->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
            external_camera_->AttachToComponent(last_parent_, FAttachmentTransformRules::KeepRelativeTransform);
        }
    }
}

void ACameraDirector::setMode(ECameraDirectorMode mode)
{
    {   //first remove any settings done by previous mode

        //detach spring arm
        if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE &&
            mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE) 
        {
            attachSpringArm(false);
        }

        // Re-enable rendering
        if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY &&
            mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY)
        {
            UAirBlueprintLib::enableViewportRendering(this, true);
        }

        //Remove any existing key bindings for manual mode
        if (mode != ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
            if (external_camera_ != nullptr
                && manual_pose_controller_->getActor() == external_camera_) {

                manual_pose_controller_->enableBindings(false);
            }
            //else someone else is bound to manual pose controller, leave it alone
        }
    }
    
    {   //perform any settings to enter in to this mode

        switch (mode) {
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL:
            //if new mode is manual mode then add key bindings
            manual_pose_controller_->enableBindings(true); break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE:
            //if we switched to spring arm mode then attach to spring arm (detachment was done earlier in method)
            attachSpringArm(true); break;
        case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY:
            UAirBlueprintLib::enableViewportRendering(this, false); break;
        default:
            //other modes don't need special setup
            break;
        }

    }

    //make switch official
    mode_ = mode;
}

void ACameraDirector::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventFpvView", EKeys::F, this, &ACameraDirector::inputEventFpvView);
    UAirBlueprintLib::BindActionToKey("inputEventFlyWithView", EKeys::B, this, &ACameraDirector::inputEventFlyWithView);
    UAirBlueprintLib::BindActionToKey("inputEventGroundView", EKeys::Backslash, this, &ACameraDirector::inputEventGroundView);
    UAirBlueprintLib::BindActionToKey("inputEventManualView", EKeys::M, this, &ACameraDirector::inputEventManualView);
    UAirBlueprintLib::BindActionToKey("inputEventSpringArmChaseView", EKeys::Slash, this, &ACameraDirector::inputEventSpringArmChaseView);
    UAirBlueprintLib::BindActionToKey("inputEventBackupView", EKeys::K, this, &ACameraDirector::inputEventBackupView);
    UAirBlueprintLib::BindActionToKey("inputEventNoDisplayView", EKeys::Hyphen, this, &ACameraDirector::inputEventNoDisplayView);
}


void ACameraDirector::inputEventFpvView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV);
    external_camera_->disableMain();
    if (backup_camera_)
        backup_camera_->disableMain();
    if (fpv_camera_)
        fpv_camera_->showToScreen();
}

void ACameraDirector::inputEventSpringArmChaseView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE);
    external_camera_->showToScreen();
    disableCameras(true, true, false);
}

void ACameraDirector::inputEventGroundView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER);
    external_camera_->showToScreen();
    disableCameras(true, true, false);
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

APIPCamera* ACameraDirector::getBackupCamera() const
{
    return backup_camera_;
}

void ACameraDirector::inputEventManualView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL);
}

void ACameraDirector::inputEventNoDisplayView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY);
    disableCameras(true, true, true);
}

void ACameraDirector::inputEventBackupView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP);
    external_camera_->disableMain();
    if (fpv_camera_)
        fpv_camera_->disableMain();
    if (backup_camera_)
        backup_camera_->showToScreen();
}

void ACameraDirector::inputEventFlyWithView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME);
    external_camera_->showToScreen();

    if (follow_actor_)
        external_camera_->SetActorLocationAndRotation(
            follow_actor_->GetActorLocation() + initial_ground_obs_offset_, camera_start_rotation_);
    disableCameras(true, true, false);
    ext_obs_fixed_z_ = false;
}

void ACameraDirector::disableCameras(bool fpv, bool backup, bool external)
{
    if (fpv && fpv_camera_)
        fpv_camera_->disableMain();
    if (backup && backup_camera_)
        backup_camera_->disableMain();
    if (external && external_camera_)
        external_camera_->disableMain();
}



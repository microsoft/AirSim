#include "CameraDirector.h"
#include "AirBlueprintLib.h"

ACameraDirector::ACameraDirector()
{
    PrimaryActorTick.bCanEverTick = true;
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
    else {
        UAirBlueprintLib::FollowActor(external_camera_, follow_actor_, initial_ground_obs_offset_, ext_obs_fixed_z_);
    }
}

ECameraDirectorMode ACameraDirector::getMode()
{
    return mode_;
}

void ACameraDirector::initializeForBeginPlay(ECameraDirectorMode view_mode, AVehiclePawnBase* vehicle, APIPCamera* external_camera)
{
    manual_pose_controller_ = NewObject<UManualPoseController>();

    setupInputBindings();

    mode_ = view_mode;
    setCameras(external_camera, vehicle);

    camera_start_location_ = this->GetActorLocation();
    camera_start_rotation_ = this->GetActorRotation();
    initial_ground_obs_offset_ = camera_start_location_ - follow_actor_->GetActorLocation();

}

void ACameraDirector::setCameras(APIPCamera* external_camera, AVehiclePawnBase* vehicle)
{
    external_camera_ = external_camera;
    fpv_camera_ = vehicle->getCamera();
    follow_actor_ = vehicle;

    manual_pose_controller_->setActor(external_camera_, false);

    //set initial view mode
    switch (mode_) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME: inputEventFlyWithView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV: inputEventFpvView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER: inputEventGroundView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL: inputEventManualView(); break;
    default:
        throw std::out_of_range("Unknown view mode specified in CameraDirector::initializeForBeginPlay");
    }
}

void ACameraDirector::setMode(ECameraDirectorMode mode)
{
    mode_ = mode;
    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL)
        manual_pose_controller_->enableBindings(true);
    else if (external_camera_ != nullptr && manual_pose_controller_->getActor() == external_camera_)
        manual_pose_controller_->enableBindings(false);
    //else someone else is bound to manual pose controller, leave it alone
}

void ACameraDirector::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventFpvView", EKeys::F, this, &ACameraDirector::inputEventFpvView);
    UAirBlueprintLib::BindActionToKey("inputEventFlyWithView", EKeys::B, this, &ACameraDirector::inputEventFlyWithView);
    UAirBlueprintLib::BindActionToKey("inputEventGroundView", EKeys::Backslash, this, &ACameraDirector::inputEventGroundView);
    UAirBlueprintLib::BindActionToKey("inputEventManualView", EKeys::Semicolon, this, &ACameraDirector::inputEventManualView);
}


void ACameraDirector::inputEventFpvView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV);
    external_camera_->disableMain();
    fpv_camera_->showToScreen();
}

void ACameraDirector::inputEventGroundView()
{
    setMode(ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER);
    external_camera_->showToScreen();
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
    fpv_camera_->disableMain();
    ext_obs_fixed_z_ = false;
}



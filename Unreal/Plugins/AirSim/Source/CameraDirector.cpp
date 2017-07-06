#include "AirSim.h"
#include "CameraDirector.h"
#include "AirBlueprintLib.h"


ACameraDirector::ACameraDirector()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ACameraDirector::BeginPlay()
{
    if (left_binding_ == nullptr) {
        setupInputBindings();
    }
    Super::BeginPlay();
}

void ACameraDirector::Tick( float DeltaTime )
{
    Super::Tick( DeltaTime );

    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
        external_camera_->SetActorLocationAndRotation(camera_location_manual_, camera_rotation_manual_);
    }
    else {
        UAirBlueprintLib::FollowActor(external_camera_, follow_actor_, initial_ground_obs_offset_, ext_obs_fixed_z_);
    }
}

ECameraDirectorMode ACameraDirector::getMode()
{
    return mode_;
}

void ACameraDirector::initializeForBeginPlay(ECameraDirectorMode view_mode)
{
    setupInputBindings();

    camera_start_location_ = this->GetActorLocation();
    camera_start_rotation_ = this->GetActorRotation();
    initial_ground_obs_offset_ = camera_start_location_ - follow_actor_->GetActorLocation();

    //set initial view mode
    switch (view_mode) {
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME: inputEventFlyWithView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV: inputEventFpvView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER: inputEventGroundView(); break;
    case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL: inputEventManualView(); break;
    default:
        throw std::out_of_range("Unknown view mode specified in CameraDirector::initializeForBeginPlay");
    }
}

void ACameraDirector::setCameras(APIPCamera* external_camera, AVehiclePawnBase* vehicle)
{
    external_camera_ = external_camera;
    fpv_camera_ = vehicle->getFpvCamera();
    follow_actor_ = vehicle;
}

void ACameraDirector::setMode(ECameraDirectorMode mode)
{
    mode_ = mode;
    if (mode_ == ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL) {
        camera_location_manual_ = external_camera_->GetActorLocation();
        camera_rotation_manual_ = external_camera_->GetActorRotation();
        enableManualBindings(true);
    }
    else
        enableManualBindings(false);
}

void ACameraDirector::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("inputEventFpvView", EKeys::LeftBracket, this, &ACameraDirector::inputEventFpvView);
    UAirBlueprintLib::BindActionToKey("inputEventFlyWithView", EKeys::RightBracket, this, &ACameraDirector::inputEventFlyWithView);
    UAirBlueprintLib::BindActionToKey("inputEventGroundView", EKeys::Backslash, this, &ACameraDirector::inputEventGroundView);
    UAirBlueprintLib::BindActionToKey("inputEventManualView", EKeys::Semicolon, this, &ACameraDirector::inputEventManualView);

    left_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualArrowLeft", EKeys::Left, this, &ACameraDirector::inputManualLeft);
    right_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualArrowRight", EKeys::Right, this, &ACameraDirector::inputManualRight);
    forward_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualForward", EKeys::Up, this, &ACameraDirector::inputManualForward);
    backward_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualBackward", EKeys::Down, this, &ACameraDirector::inputManualBackward);
    up_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualArrowUp", EKeys::PageUp, this, &ACameraDirector::inputManualMoveUp);
    down_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualArrowDown", EKeys::PageDown, this, &ACameraDirector::inputManualDown);
    left_yaw_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualLeftYaw", EKeys::A, this, &ACameraDirector::inputManualLeftYaw);
    up_pitch_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualUpPitch", EKeys::W, this, &ACameraDirector::inputManualUpPitch);
    right_yaw_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualRightYaw", EKeys::D, this, &ACameraDirector::inputManualRightYaw);
    down_pitch_binding_ = & UAirBlueprintLib::BindAxisToKey("inputManualDownPitch", EKeys::S, this, &ACameraDirector::inputManualDownPitch);
}

void ACameraDirector::enableManualBindings(bool enable)
{
    left_binding_->bConsumeInput = right_binding_->bConsumeInput = up_binding_->bConsumeInput = down_binding_->bConsumeInput = enable;
    forward_binding_->bConsumeInput = backward_binding_->bConsumeInput = left_yaw_binding_->bConsumeInput = up_pitch_binding_->bConsumeInput = enable;
    right_yaw_binding_->bConsumeInput = down_pitch_binding_->bConsumeInput = enable;
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


void ACameraDirector::inputManualLeft(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f)) {
        camera_location_manual_ += camera_rotation_manual_.RotateVector(FVector(0,-val*10,0));
    }
}
void ACameraDirector::inputManualRight(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_location_manual_ += camera_rotation_manual_.RotateVector(FVector(0, val * 10, 0));
}
void ACameraDirector::inputManualForward(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_location_manual_ += camera_rotation_manual_.RotateVector(FVector(val * 10, 0, 0));
}
void ACameraDirector::inputManualBackward(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
    camera_location_manual_ += camera_rotation_manual_.RotateVector(FVector(-val * 10, 0, 0));
}
void ACameraDirector::inputManualMoveUp(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_location_manual_ += FVector(0, 0, val * 10);
}
void ACameraDirector::inputManualDown(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_location_manual_ += FVector(0, 0, -val * 10);
}
void ACameraDirector::inputManualLeftYaw(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_rotation_manual_.Add(0, -val, 0);
}
void ACameraDirector::inputManualUpPitch(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_rotation_manual_.Add(val, 0, 0);
}
void ACameraDirector::inputManualRightYaw(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_rotation_manual_.Add(0, val, 0);
}
void ACameraDirector::inputManualDownPitch(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        camera_rotation_manual_.Add(-val, 0, 0);
}


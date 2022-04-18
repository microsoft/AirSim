#include "ManualPoseController.h"
#include "AirBlueprintLib.h"

void UManualPoseController::initializeForPlay()
{
    actor_ = nullptr;
    clearBindings();

    input_ = last_velocity_ = FVector::ZeroVector;
}

void UManualPoseController::clearBindings()
{
    left_binding_ = right_binding_ = up_binding_ = down_binding_ = nullptr;
    forward_binding_ = backward_binding_ = left_yaw_binding_ = up_pitch_binding_ = nullptr;
    right_yaw_binding_ = down_pitch_binding_ = left_roll_binding_ = right_roll_binding_ = nullptr;
    inc_speed_binding_ = dec_speed_binding_ = nullptr;
}

void UManualPoseController::setActor(AActor* actor)
{
    //if we already have attached actor
    if (actor_) {
        removeInputBindings();
    }

    actor_ = actor;

    if (actor_ != nullptr) {
        resetDelta();
        setupInputBindings();
    }
}

AActor* UManualPoseController::getActor() const
{
    return actor_;
}

void UManualPoseController::updateActorPose(float dt)
{
    if (actor_ != nullptr) {
        updateDeltaPosition(dt);

        FVector location = actor_->GetActorLocation();
        FRotator rotation = actor_->GetActorRotation();
        actor_->SetActorLocationAndRotation(location + delta_position_, rotation + delta_rotation_);
        resetDelta();
    }
    else {
        UAirBlueprintLib::LogMessageString("UManualPoseController::updateActorPose should not be called when actor is not set", "", LogDebugLevel::Failure);
    }
}

void UManualPoseController::getDeltaPose(FVector& delta_position, FRotator& delta_rotation) const
{
    delta_position = delta_position_;
    delta_rotation = delta_rotation_;
}

void UManualPoseController::resetDelta()
{
    delta_position_ = FVector::ZeroVector;
    delta_rotation_ = FRotator::ZeroRotator;
}

void UManualPoseController::removeInputBindings()
{
        UAirBlueprintLib::DisableInput(actor_);
}

void UManualPoseController::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(actor_);

    UAirBlueprintLib::BindAxis("inputManualArrowRight", getActor(), this, &UManualPoseController::inputManualMoveRight);
    UAirBlueprintLib::BindAxis("inputManualForward", getActor(), this, &UManualPoseController::inputManualMoveForward);
    UAirBlueprintLib::BindAxis("inputManualArrowUp", getActor(), this, &UManualPoseController::inputManualMoveUp);
    UAirBlueprintLib::BindAxis("inputManualRightYaw", getActor(), this, &UManualPoseController::inputManualYaw);
    UAirBlueprintLib::BindAxis("inputManualRightRoll", getActor(), this, &UManualPoseController::inputManualRoll);
    UAirBlueprintLib::BindAxis("inputManualUpPitch", getActor(), this, &UManualPoseController::inputManualPitch);
    UAirBlueprintLib::BindAxis("inputManualSpeedIncrease", getActor(), this, &UManualPoseController::inputManualSpeedChange);
}

void UManualPoseController::updateDeltaPosition(float dt)
{
    FVector input = input_;
    if (!FMath::IsNearlyZero(input.SizeSquared())) {
        if (FMath::IsNearlyZero(acceleration_))
            last_velocity_ = input * speed_scaler_;
        else
            last_velocity_ += input * (acceleration_ * dt);
        delta_position_ += actor_->GetActorRotation().RotateVector(last_velocity_ * dt);
    }
    else {
        delta_position_ = last_velocity_ = FVector::ZeroVector;
    }
}

void UManualPoseController::inputManualSpeedChange(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        speed_scaler_ += val * 20;

    if (speed_scaler_ <= 0.0)
        speed_scaler_ = 20.0;
}

void UManualPoseController::inputManualMoveRight(float val)
{
    input_.Y = val;
}
void UManualPoseController::inputManualMoveForward(float val)
{
    input_.X = val;
}
void UManualPoseController::inputManualMoveUp(float val)
{
    input_.Z = val;
}

void UManualPoseController::inputManualYaw(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, val, 0);
}
void UManualPoseController::inputManualRoll(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, 0, val);
}
void UManualPoseController::inputManualPitch(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(val, 0, 0);
}

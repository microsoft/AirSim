#include "ManualPoseController.h"
#include "AirBlueprintLib.h"


void UManualPoseController::initializeForPlay()
{
    actor_ = last_actor_ = nullptr;
    left_binding_ = right_binding_ = up_binding_ = down_binding_ = nullptr;
    forward_binding_ = backward_binding_ = left_yaw_binding_ = up_pitch_binding_ = nullptr;
    right_yaw_binding_ = down_pitch_binding_ = nullptr;

    left_mapping_ = FInputAxisKeyMapping("inputManualArrowLeft", EKeys::Left); right_mapping_ = FInputAxisKeyMapping("inputManualArrowRight", EKeys::Right); 
    forward_mapping_= FInputAxisKeyMapping("inputManualForward", EKeys::Up); backward_mapping_ = FInputAxisKeyMapping("inputManualBackward", EKeys::Down);
    up_mapping_ = FInputAxisKeyMapping("inputManualArrowUp", EKeys::PageUp); down_mapping_ = FInputAxisKeyMapping("inputManualArrowDown", EKeys::PageDown); 
    left_yaw_mapping_ = FInputAxisKeyMapping("inputManualLeftYaw", EKeys::A); up_pitch_mapping_ = FInputAxisKeyMapping("inputManualUpPitch", EKeys::W);
    right_yaw_mapping_ = FInputAxisKeyMapping("inputManualRightYaw", EKeys::D); down_pitch_mapping_ = FInputAxisKeyMapping("inputManualDownPitch", EKeys::S);
}

void UManualPoseController::restoreLastActor()
{
    setActor(last_actor_);
}

void UManualPoseController::setActor(AActor* actor, bool enable_binding)
{
    //TODO: can't do remove because there is no "stamp" on who established binding
    //removeInputBindings();

    last_actor_ = actor_;
    actor_ = actor;

    if (actor_ != nullptr) {
        resetDelta();
        setupInputBindings();
        enableBindings(enable_binding);
    }
}

AActor* UManualPoseController::getActor() const
{
    return actor_;
}

void UManualPoseController::updateActorPose()
{
    if (actor_ != nullptr) {
        FVector location = actor_->GetActorLocation();
        FRotator rotation = actor_->GetActorRotation();
        actor_->SetActorLocationAndRotation(location + delta_position_, rotation + delta_rotation_);
        resetDelta();
    }
    else {
        UAirBlueprintLib::LogMessageString("UManualPoseController::updateActorPose should not be called when actor is not set", "", LogDebugLevel::Failure);
    }
}

void UManualPoseController::getActorDeltaPose(FVector& delta_position, FRotator& delta_rotation, bool reset_delta)
{
    delta_position = delta_position_;
    delta_rotation = delta_rotation_;

    if (reset_delta)
        resetDelta();
}

void UManualPoseController::resetDelta()
{
    delta_position_ = FVector::ZeroVector;
    delta_rotation_ = FRotator::ZeroRotator;
}

void UManualPoseController::removeInputBindings()
{
    UAirBlueprintLib::RemoveAxisBinding(left_mapping_, left_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(right_mapping_, right_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(forward_mapping_, forward_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(backward_mapping_, backward_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(up_mapping_, up_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(down_mapping_, down_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(left_yaw_mapping_, left_yaw_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(up_pitch_mapping_, up_pitch_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(right_yaw_mapping_, right_yaw_binding_, actor_);
    UAirBlueprintLib::RemoveAxisBinding(down_pitch_mapping_, down_pitch_binding_, actor_);
}

void UManualPoseController::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(actor_);


    left_binding_ = & UAirBlueprintLib::BindAxisToKey(left_mapping_, actor_, this, &UManualPoseController::inputManualLeft);
    right_binding_ = & UAirBlueprintLib::BindAxisToKey(right_mapping_, actor_, this, &UManualPoseController::inputManualRight);
    forward_binding_ = & UAirBlueprintLib::BindAxisToKey(forward_mapping_, actor_, this, &UManualPoseController::inputManualForward);
    backward_binding_ = & UAirBlueprintLib::BindAxisToKey(backward_mapping_, actor_, this, &UManualPoseController::inputManualBackward);
    up_binding_ = & UAirBlueprintLib::BindAxisToKey(up_mapping_, actor_, this, &UManualPoseController::inputManualMoveUp);
    down_binding_ = & UAirBlueprintLib::BindAxisToKey(down_mapping_, actor_, this, &UManualPoseController::inputManualDown);
    left_yaw_binding_ = & UAirBlueprintLib::BindAxisToKey(left_yaw_mapping_, actor_, this, &UManualPoseController::inputManualLeftYaw);
    up_pitch_binding_ = & UAirBlueprintLib::BindAxisToKey(up_pitch_mapping_, actor_, this, &UManualPoseController::inputManualUpPitch);
    right_yaw_binding_ = & UAirBlueprintLib::BindAxisToKey(right_yaw_mapping_, actor_, this, &UManualPoseController::inputManualRightYaw);
    down_pitch_binding_ = & UAirBlueprintLib::BindAxisToKey(down_pitch_mapping_, actor_, this, &UManualPoseController::inputManualDownPitch);
}

void UManualPoseController::enableBindings(bool enable)
{
    left_binding_->bConsumeInput = right_binding_->bConsumeInput = up_binding_->bConsumeInput = down_binding_->bConsumeInput = enable;
    forward_binding_->bConsumeInput = backward_binding_->bConsumeInput = left_yaw_binding_->bConsumeInput = up_pitch_binding_->bConsumeInput = enable;
    right_yaw_binding_->bConsumeInput = down_pitch_binding_->bConsumeInput = enable;
}

void UManualPoseController::inputManualLeft(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f)) {
        delta_position_ += actor_->GetActorRotation().RotateVector(FVector(0,-val*10,0));
    }
}
void UManualPoseController::inputManualRight(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_position_ += actor_->GetActorRotation().RotateVector(FVector(0, val * 10, 0));
}
void UManualPoseController::inputManualForward(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_position_ += actor_->GetActorRotation().RotateVector(FVector(val * 10, 0, 0));
}
void UManualPoseController::inputManualBackward(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_position_ += actor_->GetActorRotation().RotateVector(FVector(-val * 10, 0, 0));
}
void UManualPoseController::inputManualMoveUp(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_position_ += actor_->GetActorRotation().RotateVector(FVector(0, 0, val * 10));
}
void UManualPoseController::inputManualDown(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_position_ += actor_->GetActorRotation().RotateVector(FVector(0, 0, -val * 10));
}
void UManualPoseController::inputManualLeftYaw(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, -val, 0);
}
void UManualPoseController::inputManualUpPitch(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(val, 0, 0);
}
void UManualPoseController::inputManualRightYaw(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(0, val, 0);
}
void UManualPoseController::inputManualDownPitch(float val)
{
    if (!FMath::IsNearlyEqual(val, 0.f))
        delta_rotation_.Add(-val, 0, 0);
}

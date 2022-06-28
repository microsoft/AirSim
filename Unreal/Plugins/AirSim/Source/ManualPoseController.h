#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerInput.h"

#include "ManualPoseController.generated.h"

UCLASS()
class AIRSIM_API UManualPoseController : public UObject
{
    GENERATED_BODY()

public:
    void initializeForPlay();
    void setActor(AActor* actor);
    AActor* getActor() const;
    void updateActorPose(float dt);
    void getDeltaPose(FVector& delta_position, FRotator& delta_rotation) const;
    void resetDelta();
    void updateDeltaPosition(float dt);

private:
    void inputManualMoveForward(float val);
    void inputManualMoveRight(float val);
    void inputManualMoveUp(float val);
    void inputManualYaw(float val);
    void inputManualRoll(float val);
    void inputManualPitch(float val);
    void inputManualSpeedChange(float val);

    void setupInputBindings();
    void removeInputBindings();
    void clearBindings();

private:
    FInputAxisBinding *left_binding_, *right_binding_, *up_binding_, *down_binding_;
    FInputAxisBinding *forward_binding_, *backward_binding_, *left_yaw_binding_, *right_yaw_binding_;
    FInputAxisBinding *up_pitch_binding_, *down_pitch_binding_, *left_roll_binding_, *right_roll_binding_;
    FInputAxisBinding *inc_speed_binding_, *dec_speed_binding_;

    FInputAxisKeyMapping left_mapping_, right_mapping_, up_mapping_, down_mapping_;
    FInputAxisKeyMapping forward_mapping_, backward_mapping_, left_yaw_mapping_, right_yaw_mapping_;
    FInputAxisKeyMapping up_pitch_mapping_, down_pitch_mapping_, left_roll_mapping_, right_roll_mapping_;
    FInputAxisKeyMapping inc_speed_mapping_, dec_speed_mapping_;

    FVector delta_position_;
    FRotator delta_rotation_;

    AActor* actor_;

    float acceleration_ = 0, speed_scaler_ = 1000;
    FVector input_;
    FVector last_velocity_;
};
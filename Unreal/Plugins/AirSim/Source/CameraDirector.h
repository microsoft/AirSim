#pragma once

#include "VehiclePawnBase.h"
#include "PIPCamera.h"
#include "GameFramework/Actor.h"
#include "CameraDirector.generated.h"


UENUM(BlueprintType)
enum class ECameraDirectorMode : uint8
{
    CAMERA_DIRECTOR_MODE_FPV = 1	UMETA(DisplayName="FPV"),
    CAMERA_DIRECTOR_MODE_GROUND_OBSERVER = 2	UMETA(DisplayName="GroundObserver"),
    CAMERA_DIRECTOR_MODE_FLY_WITH_ME = 3	UMETA(DisplayName="FlyWithMe"),
    CAMERA_DIRECTOR_MODE_MANUAL = 4	UMETA(DisplayName="Manual")
};

UCLASS()
class AIRSIM_API ACameraDirector : public AActor
{
    GENERATED_BODY()
    
public:
    //below should be set by SimMode BP
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pawn")
    AVehiclePawnBase* TargetPawn;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pawn")
    APIPCamera* ExternalCamera;

    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventFpvView"))
    void InputEventFpvView();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventGroundView"))
    void InputEventGroundView();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventManualView"))
    void InputEventManualView();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventFlyWithView"))
    void InputEventFlyWithView();

    UFUNCTION(BlueprintCallable, Category = "PIP")
    bool togglePIPScene();
    UFUNCTION(BlueprintCallable, Category = "PIP")
    bool togglePIPDepth();
    UFUNCTION(BlueprintCallable, Category = "PIP")
    bool togglePIPSeg();
    UFUNCTION(BlueprintCallable, Category = "PIP")
    bool togglePIPAll();

    UFUNCTION(BlueprintCallable, Category = "PIP")
    APIPCamera* getCamera(int id = 0);

public:	
    ACameraDirector();
    virtual void BeginPlay() override;
    virtual void Tick( float DeltaSeconds ) override;

    UFUNCTION(BlueprintCallable, Category = "Modes")
    ECameraDirectorMode getMode();
    UFUNCTION(BlueprintCallable, Category = "Modes")
    void setMode(ECameraDirectorMode mode);

private:
    void setupInputBindings();	
    bool checkCameraRefs();
    void enableManualBindings(bool enable);

    void inputManualLeft(float val);
    void inputManualRight(float val);
    void inputManualForward(float val);
    void inputManualBackward(float val);
    void inputManualMoveUp(float val);
    void inputManualDown(float val);
    void inputManualLeftYaw(float val);
    void inputManualUpPitch(float val);
    void inputManualRightYaw(float val);
    void inputManualDownPitch(float val);

private:
    ECameraDirectorMode mode_;
    FInputAxisBinding *left_binding_, *right_binding_, *up_binding_, *down_binding_;
    FInputAxisBinding *forward_binding_, *backward_binding_, *left_yaw_binding_, *up_pitch_binding_;
    FInputAxisBinding *right_yaw_binding_, *down_pitch_binding_;

    FVector camera_location_manual_;
    FRotator camera_rotation_manual_;
};

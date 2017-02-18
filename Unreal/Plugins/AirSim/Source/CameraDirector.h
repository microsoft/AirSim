#pragma once

#include "VehiclePawnBase.h"
#include "PIPCamera.h"
#include "GameFramework/Actor.h"
#include "CameraDirector.generated.h"

UCLASS()
class AIRSIM_API ACameraDirector : public AActor {
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

  private:
    void setupInputBindings();
    bool checkCameraRefs();
};

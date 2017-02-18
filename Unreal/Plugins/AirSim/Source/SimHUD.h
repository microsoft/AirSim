#pragma once

#include "GameFramework/HUD.h"
#include "SimHUD.generated.h"


UENUM(BlueprintType)
enum class ESimulatorMode : uint8
{
    SIM_MODE_HIL 	UMETA(DisplayName="Hardware-in-loop")
};

UCLASS()
class AIRSIM_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventToggleReport"))
    void InputEventToggleReport();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventToggleHelp"))
    void InputEventToggleHelp();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventToggleTrace"))
    void InputEventToggleTrace();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventTogglePIPScene"))
    void InputEventTogglePIPScene();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventTogglePIPDepth"))
    void InputEventTogglePIPDepth();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventTogglePIPSeg"))
    void InputEventTogglePIPSeg();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "InputEventToggleAll"))
    void InputEventToggleAll();

    virtual void BeginPlay() override;

protected:
    virtual void setupInputBindings();
};

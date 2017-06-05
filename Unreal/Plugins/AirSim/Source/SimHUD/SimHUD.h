#pragma once

#include "GameFramework/HUD.h"
#include "SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
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
    void inputEventToggleReport();
    void inputEventToggleHelp();
    void inputEventToggleTrace();
    void inputEventTogglePIPScene();
    void inputEventTogglePIPDepth();
    void inputEventTogglePIPSeg();
    void inputEventToggleAll();

    bool isPIPSceneVisible();
    bool isPIPDepthVisible();
    bool isPIPSegVisible();

    ASimHUD();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick( float DeltaSeconds ) override;

    static ASimHUD* GetInstance() {
        return instance_;
    }
protected:
    virtual void setupInputBindings();
    std::string reportRefreshHandler();
    void toggleRecordHandler();

private:
    UClass* widget_class_;

    UPROPERTY()
    USimHUDWidget* widget_;
    UPROPERTY()
    ASimModeBase* simmode_;

    static ASimHUD* instance_;
};

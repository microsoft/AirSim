#pragma once

#include "GameFramework/HUD.h"
#include "SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
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
    void inputEventToggleSubwindow0();
    void inputEventToggleSubwindow1();
    void inputEventToggleSubwindow2();
    void inputEventToggleAll();


    EPIPCameraType getSubwindowCameraType(int window_index);
    void setSubwindowCameraType(int window_index, EPIPCameraType type);
    APIPCamera* getSubwindowCamera(int window_index);
    void setSubwindowCamera(int window_index, APIPCamera* camera);
    bool getSubwindowVisible(int window_index);
    void setSubwindowVisible(int window_index, bool is_visible);
        
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
    void updateWidgetSubwindowVisibility();
    bool isWidgetSubwindowVisible(int window_index);
private:
    UClass* widget_class_;

    UPROPERTY() USimHUDWidget* widget_;
    UPROPERTY() ASimModeBase* simmode_;

    static constexpr int kSubwindowCount = 3; //must be >= 3 for now
    APIPCamera* subwindow_cameras_[kSubwindowCount];
    EPIPCameraType subwindow_camera_types_[kSubwindowCount];
    bool subwindow_visible_[kSubwindowCount];

    static ASimHUD* instance_;
};

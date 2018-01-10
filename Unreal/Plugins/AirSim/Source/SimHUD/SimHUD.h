#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
#include "SimHUD.generated.h"


UENUM(BlueprintType)
enum class ESimulatorMode : uint8
{
    SIM_MODE_HIL 	UMETA(DisplayName = "Hardware-in-loop")
};

UCLASS()
class AIRSIM_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    void inputEventToggleRecording();
    void inputEventToggleReport();
    void inputEventToggleHelp();
    void inputEventToggleTrace();
    void inputEventToggleSubwindow0();
    void inputEventToggleSubwindow1();
    void inputEventToggleSubwindow2();
    void inputEventToggleAll();


    ImageType getSubwindowCameraType(int window_index);
    void setSubwindowCameraType(int window_index, ImageType type);
    APIPCamera* getSubwindowCamera(int window_index);
    void setSubwindowCamera(int window_index, APIPCamera* camera);
    bool getSubwindowVisible(int window_index);
    void setSubwindowVisible(int window_index, bool is_visible);

    ASimHUD();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

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
    void initializeSubWindows();
    void createSimMode();
    void initializeSettings();
    void setUnrealEngineSettings();
    void createMainWidget();
    const std::vector<AirSimSettings::SubwindowSetting>& getSubWindowSettings() const;
    std::vector<AirSimSettings::SubwindowSetting>& getSubWindowSettings();
    

    bool getSettingsText(std::string& settingsText);
    bool getSettingsTextFromCommandLine(std::string& settingsText);
    bool readSettingsTextFromFile(FString fileName, std::string& settingsText);
    std::string getSimModeFromUser();


private:
    typedef common_utils::Utils Utils;
    UClass* widget_class_;

    UPROPERTY() USimHUDWidget* widget_;
    UPROPERTY() ASimModeBase* simmode_;

    APIPCamera* subwindow_cameras_[AirSimSettings::kSubwindowCount];

    static ASimHUD* instance_;
};

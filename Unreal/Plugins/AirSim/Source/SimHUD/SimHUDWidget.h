#pragma once

#include "Blueprint/UserWidget.h"
#include "PIPCamera.h"
#include <functional>
#include "SimHUDWidget.generated.h"


UCLASS()
class AIRSIM_API USimHUDWidget : public UUserWidget
{
    GENERATED_BODY()
    
public:
    UFUNCTION(BlueprintCallable, Category = "Event handler")
        void onToggleRecordingButtonClick();

public:
    typedef std::function<void (void)> OnToggleRecording;


    //TODO: Tick is not working
    //virtual void Tick_Implementation(FGeometry MyGeometry, float InDeltaTime) override;
    
    void updateReport(const std::string& text);
    void setReportVisible(bool is_visible);
    void toggleHelpVisibility();
    void refreshPIPVisibility(APIPCamera* camera);

    void setOnToggleRecordingHandler(OnToggleRecording handler);


public:
    //below are implemented in Blueprint. The return value is forced to be
    //bool even when not needed because of Unreal quirk that if return value
    //is not there then below are treated as events instead of overridable functions
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setPIPSceneVisibility(bool is_viisble);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool getPIPSceneVisibility();
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setPIPDepthVisibility(bool is_viisble);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool getPIPDepthVisibility();
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setPIPSegVisibility(bool is_viisble);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool getPIPSegVisibility();
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setRecordButtonVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool getRecordButtonVisibility();
protected:
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setReportContainerVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool getReportContainerVisibility();
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setHelpContainerVisibility(bool is_visible);
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool getHelpContainerVisibility();
    UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
        bool setReportText(const FString& text);

private:
    OnToggleRecording on_toggle_recording_;
};

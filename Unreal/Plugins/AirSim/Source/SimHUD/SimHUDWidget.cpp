#include "SimHUDWidget.h"

void USimHUDWidget::updateDebugReport(const std::string& text)
{
    setReportText(FString(text.c_str()));
}

void USimHUDWidget::setReportVisible(bool is_visible)
{
    setReportContainerVisibility(is_visible);
}

void USimHUDWidget::toggleHelpVisibility()
{
    setHelpContainerVisibility(!getHelpContainerVisibility());
}

void USimHUDWidget::setOnToggleRecordingHandler(OnToggleRecording handler)
{
    on_toggle_recording_ = handler;
}

void USimHUDWidget::onToggleRecordingButtonClick()
{
    on_toggle_recording_();
}
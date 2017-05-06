#include "AirSim.h"
#include "SimHUDWidget.h"

void USimHUDWidget::updateReport(const std::string& text)
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

void USimHUDWidget::refreshPIPVisibility(APIPCamera* camera)
{
    EPIPCameraType enabled_cameras = camera->getEnableCameraTypes();

    setPIPSceneVisibility((enabled_cameras & EPIPCameraType::PIP_CAMERA_TYPE_SCENE) != EPIPCameraType::PIP_CAMERA_TYPE_NONE);
    setPIPDepthVisibility((enabled_cameras & EPIPCameraType::PIP_CAMERA_TYPE_DEPTH) != EPIPCameraType::PIP_CAMERA_TYPE_NONE);
    setPIPSegVisibility((enabled_cameras & EPIPCameraType::PIP_CAMERA_TYPE_SEG) != EPIPCameraType::PIP_CAMERA_TYPE_NONE);
}

void USimHUDWidget::setOnToggleRecordingHandler(OnToggleRecording handler)
{
    on_toggle_recording_ = handler;
}

void USimHUDWidget::onToggleRecordingButtonClick()
{
    on_toggle_recording_();
}
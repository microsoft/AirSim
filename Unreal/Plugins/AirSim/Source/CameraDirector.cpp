#include "AirSim.h"
#include "CameraDirector.h"
#include "AirBlueprintLib.h"


ACameraDirector::ACameraDirector()
{
    PrimaryActorTick.bCanEverTick = true;

}

void ACameraDirector::BeginPlay()
{
    Super::BeginPlay();

    setupInputBindings();
}

void ACameraDirector::Tick( float DeltaTime )
{
    Super::Tick( DeltaTime );
}

void ACameraDirector::setupInputBindings()
{
    this->EnableInput(this->GetWorld()->GetFirstPlayerController());

    UAirBlueprintLib::BindActionTokey("InputEventFpvView", EKeys::LeftBracket, this, &ACameraDirector::InputEventFpvView);
    UAirBlueprintLib::BindActionTokey("InputEventFlyWithView", EKeys::RightBracket, this, &ACameraDirector::InputEventFlyWithView);
    UAirBlueprintLib::BindActionTokey("InputEventGroundView", EKeys::Backslash, this, &ACameraDirector::InputEventGroundView);
}

bool ACameraDirector::checkCameraRefs()
{
    if (ExternalCamera == nullptr || TargetPawn == nullptr || TargetPawn->getFpvCamera() == nullptr) {
        UAirBlueprintLib::LogMessage("Cannot toggle PIP camera because FPV pwn camera and/or external camera is not set", "", LogDebugLevel::Failure, 60);
        return false;
    }
    return true;
}

bool ACameraDirector::togglePIPScene()
{
    if (!checkCameraRefs())
        return false;
    EPIPCameraType main_state = ExternalCamera->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_SCENE);
    EPIPCameraType pip_state = TargetPawn->getFpvCamera()->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_SCENE);

    if (ExternalCamera->getCameraMode() == EPIPCameraMode::PIP_CAMERA_MODE_PIP)
        return main_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
    else
        return pip_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
}

bool ACameraDirector::togglePIPDepth()
{
    if (!checkCameraRefs())
        return false;
    EPIPCameraType main_state = ExternalCamera->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_DEPTH);
    EPIPCameraType pip_state = TargetPawn->getFpvCamera()->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_DEPTH);

    if (ExternalCamera->getCameraMode() == EPIPCameraMode::PIP_CAMERA_MODE_PIP)
        return main_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
    else
        return pip_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
}

bool ACameraDirector::togglePIPSeg()
{
    if (!checkCameraRefs())
        return false;
    EPIPCameraType main_state = ExternalCamera->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_SEG);
    EPIPCameraType pip_state = TargetPawn->getFpvCamera()->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_SEG);

    if (ExternalCamera->getCameraMode() == EPIPCameraMode::PIP_CAMERA_MODE_PIP)
        return main_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
    else
        return pip_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
}

bool ACameraDirector::togglePIPAll()
{
    if (!checkCameraRefs())
        return false;
    EPIPCameraType main_state = ExternalCamera->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_ALL);
    EPIPCameraType pip_state = TargetPawn->getFpvCamera()->toggleEnableCameraTypes(EPIPCameraType::PIP_CAMERA_TYPE_ALL);

    if (ExternalCamera->getCameraMode() == EPIPCameraMode::PIP_CAMERA_MODE_PIP)
        return main_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
    else
        return pip_state != EPIPCameraType::PIP_CAMERA_TYPE_NONE;
}


APIPCamera* ACameraDirector::getCamera(int id)
{
    //TODO: support multiple camera
    if (TargetPawn != nullptr)
        return TargetPawn->getFpvCamera();
    else
        return nullptr;
}

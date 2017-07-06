#include "AirSim.h"
#include "SimHUD.h"
#include "SimMode/SimModeWorldMultiRotor.h"
#include "Kismet/KismetSystemLibrary.h"

ASimHUD* ASimHUD::instance_ = nullptr;

ASimHUD::ASimHUD()
{
    static ConstructorHelpers::FClassFinder<UUserWidget> hud_widget_class(TEXT("WidgetBlueprint'/AirSim/Blueprints/BP_SimHUDWidget'"));
    widget_class_ = hud_widget_class.Succeeded() ? hud_widget_class.Class : nullptr;
    instance_ = this;
}

void ASimHUD::BeginPlay()
{
    Super::BeginPlay();

    //Equivalent to enabling Custom Stencil in Project > Settings > Rendering > Postprocessing
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(), FString("r.CustomDepth 3"));

    //create main widget
    if (widget_class_ != nullptr) {
        widget_ = CreateWidget<USimHUDWidget>(this->GetOwningPlayerController(), widget_class_);
    }
    else {
        widget_ = nullptr;
        UAirBlueprintLib::LogMessage(TEXT("Cannot instantiate BP_SimHUDWidget blueprint!"), TEXT(""), LogDebugLevel::Failure, 180);
    }

    //create simmode
    FActorSpawnParameters simmode_spawn_params;
    simmode_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    simmode_ = this->GetWorld()->SpawnActor<ASimModeWorldMultiRotor>(FVector::ZeroVector, FRotator::ZeroRotator, simmode_spawn_params);

    subwindow_cameras_[0] = subwindow_cameras_[1] = subwindow_cameras_[2] = simmode_->getFpvVehiclePawn()->getFpvCamera();
    subwindow_camera_types_[0] = EPIPCameraType::PIP_CAMERA_TYPE_DEPTH;
    subwindow_camera_types_[1] = EPIPCameraType::PIP_CAMERA_TYPE_SEG;
    subwindow_camera_types_[2] = EPIPCameraType::PIP_CAMERA_TYPE_SCENE;
    subwindow_visible_[0] = subwindow_visible_[1] = subwindow_visible_[2] = false;

    setupInputBindings();

    widget_->AddToViewport();

    //synchronize PIP views
    widget_->initializeForPlay();
    widget_->setReportVisible(simmode_->EnableReport);
    widget_->setOnToggleRecordingHandler(std::bind(&ASimHUD::toggleRecordHandler, this));
    widget_->setRecordButtonVisibility(simmode_->isRecordUIVisible());
    updateWidgetSubwindowVisibility();
}

void ASimHUD::Tick( float DeltaSeconds )
{
    if (simmode_->EnableReport)
        widget_->updateReport(simmode_->getReport());
}

void ASimHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (widget_) {
        widget_->Destruct();
        widget_ = nullptr;
    }
    if (simmode_) {
        simmode_->Destroy();
        simmode_ = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

void ASimHUD::toggleRecordHandler()
{
    simmode_->toggleRecording();
}

void ASimHUD::inputEventToggleReport()
{
    simmode_->EnableReport = !simmode_->EnableReport;
    widget_->setReportVisible(simmode_->EnableReport);
}

void ASimHUD::inputEventToggleHelp()
{
    widget_->toggleHelpVisibility();
}

void ASimHUD::inputEventToggleTrace()
{
    simmode_->getFpvVehiclePawn()->toggleTrace();
}

EPIPCameraType ASimHUD::getSubwindowCameraType(int window_index)
{
    return subwindow_camera_types_[window_index]; //TODO: index check
}
void ASimHUD::setSubwindowCameraType(int window_index, EPIPCameraType type)
{
    subwindow_camera_types_[window_index] = type;
    updateWidgetSubwindowVisibility();
}

APIPCamera* ASimHUD::getSubwindowCamera(int window_index)
{
    return subwindow_cameras_[window_index]; //TODO: index check
}
void ASimHUD::setSubwindowCamera(int window_index, APIPCamera* camera)
{
    subwindow_cameras_[window_index] = camera; //TODO: index check
    updateWidgetSubwindowVisibility();
}

bool ASimHUD::getSubwindowVisible(int window_index)
{
    return subwindow_visible_[window_index];
}

void ASimHUD::setSubwindowVisible(int window_index, bool is_visible)
{
    subwindow_visible_[window_index] = is_visible;
    updateWidgetSubwindowVisibility();
}

void ASimHUD::updateWidgetSubwindowVisibility()
{
    for (int window_index = 0; window_index < kSubwindowCount; ++window_index) {
        APIPCamera* camera = subwindow_cameras_[window_index];
        EPIPCameraType camera_type = subwindow_camera_types_[window_index];

        bool is_visible = subwindow_visible_[window_index] && camera != nullptr &&
            camera_type != EPIPCameraType::PIP_CAMERA_TYPE_NONE;

        if (camera != nullptr) {
            if (is_visible)
                camera->setEnableCameraTypes(camera->getEnableCameraTypes() | camera_type);
            else
                camera->setEnableCameraTypes(camera->getEnableCameraTypes() & (~camera_type));
        }

        widget_->setSubwindowVisibility(window_index, 
            is_visible,
            is_visible ? camera->getRenderTarget(camera_type, false) : nullptr
        );
    }
}

bool ASimHUD::isWidgetSubwindowVisible(int window_index)
{
    return widget_->getSubwindowVisibility(window_index) != 0;
}

void ASimHUD::inputEventToggleSubwindow0()
{
    subwindow_visible_[0] = !subwindow_visible_[0];
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleSubwindow1()
{
    subwindow_visible_[1] = !subwindow_visible_[1];
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleSubwindow2()
{
    subwindow_visible_[2] = !subwindow_visible_[2];
    updateWidgetSubwindowVisibility();
}

void ASimHUD::inputEventToggleAll()
{
    subwindow_visible_[0] = !subwindow_visible_[0];
    subwindow_visible_[1] = subwindow_visible_[2] = subwindow_visible_[0];
    updateWidgetSubwindowVisibility();
}


void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::BindActionToKey("InputEventToggleReport", EKeys::R, this, &ASimHUD::inputEventToggleReport);
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::inputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey("InputEventToggleTrace", EKeys::T, this, &ASimHUD::inputEventToggleTrace);

    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow0", EKeys::One, this, &ASimHUD::inputEventToggleSubwindow0);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow1", EKeys::Two, this, &ASimHUD::inputEventToggleSubwindow1);
    UAirBlueprintLib::BindActionToKey("InputEventToggleSubwindow2", EKeys::Three, this, &ASimHUD::inputEventToggleSubwindow2);
    UAirBlueprintLib::BindActionToKey("InputEventToggleAll", EKeys::Zero, this, &ASimHUD::inputEventToggleAll);
}
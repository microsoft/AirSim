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

    setupInputBindings();

    widget_->AddToViewport();

    //synchronize PIP views
    widget_->setReportVisible(simmode_->EnableReport);
    widget_->refreshPIPVisibility(simmode_->CameraDirector->getCamera());
    widget_->setOnToggleRecordingHandler(std::bind(&ASimHUD::toggleRecordHandler, this));
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
    simmode_->CameraDirector->TargetPawn->toggleTrace();
}


bool ASimHUD::isPIPSceneVisible()
{
    return widget_->getPIPSceneVisibility();
}

bool ASimHUD::isPIPDepthVisible()
{
    return widget_->getPIPDepthVisibility();
}

bool ASimHUD::isPIPSegVisible()
{
    return widget_->getPIPSegVisibility();
}


void ASimHUD::inputEventTogglePIPScene()
{
    bool is_visible = simmode_->CameraDirector->togglePIPScene();
    widget_->setPIPSceneVisibility(is_visible);
}

void ASimHUD::inputEventTogglePIPDepth()
{
    bool is_visible = simmode_->CameraDirector->togglePIPDepth();
    widget_->setPIPDepthVisibility(is_visible);
}

void ASimHUD::inputEventTogglePIPSeg()
{
    bool is_visible = simmode_->CameraDirector->togglePIPSeg();
    widget_->setPIPSegVisibility(is_visible);
}

void ASimHUD::inputEventToggleAll()
{
    bool is_visible = simmode_->CameraDirector->togglePIPAll();
    widget_->setPIPSceneVisibility(is_visible);
    widget_->setPIPDepthVisibility(is_visible);
    widget_->setPIPSegVisibility(is_visible);
}


void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::BindActionToKey("InputEventToggleReport", EKeys::R, this, &ASimHUD::inputEventToggleReport);
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::inputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey("InputEventToggleTrace", EKeys::T, this, &ASimHUD::inputEventToggleTrace);
    
    UAirBlueprintLib::BindActionToKey("InputEventTogglePIPScene", EKeys::Three, this, &ASimHUD::inputEventTogglePIPScene);
    UAirBlueprintLib::BindActionToKey("InputEventTogglePIPDepth", EKeys::One, this, &ASimHUD::inputEventTogglePIPDepth);
    UAirBlueprintLib::BindActionToKey("InputEventTogglePIPSeg", EKeys::Two, this, &ASimHUD::inputEventTogglePIPSeg);
    UAirBlueprintLib::BindActionToKey("InputEventToggleAll", EKeys::Zero, this, &ASimHUD::inputEventToggleAll);
}
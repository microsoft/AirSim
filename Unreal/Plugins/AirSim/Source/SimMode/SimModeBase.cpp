#include "SimModeBase.h"
#include "Misc/MessageDialog.h"
#include "Misc/EngineVersion.h"
#include "AirBlueprintLib.h"
#include "Runtime/Launch/Resources/Version.h"
#include "common/AirSimSettings.hpp"
#include "Recording/RecordingThread.h"
#include "SimJoyStick/SimJoyStick.h"

ASimModeBase::ASimModeBase()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    float clock_speed = getSettings().clock_speed;

    if (clock_speed != 1.0f) {
        this->GetWorldSettings()->SetTimeDilation(clock_speed);
        UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(clock_speed), LogDebugLevel::Informational);
    }

    setStencilIDs();

    record_tick_count = 0;
    setupInputBindings();

    UAirBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""), LogDebugLevel::Informational);
}

void ASimModeBase::setStencilIDs()
{
    UAirBlueprintLib::InitializeMeshStencilIDs();
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FRecordingThread::stopRecording();
    Super::EndPlay(EndPlayReason);
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (isRecording())
        ++record_tick_count;
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::reset()
{
    //Should be overridden by derived classes
}

VehiclePawnWrapper* ASimModeBase::getFpvVehiclePawnWrapper()
{
    //Should be overridden by derived classes
    return nullptr;
}


std::string ASimModeBase::getReport()
{
    static const std::string empty_string = std::string();
    //Should be overridden by derived classes
    return empty_string;
}

void ASimModeBase::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("InputEventResetAll", EKeys::BackSpace, this, &ASimModeBase::reset);
}

bool ASimModeBase::isRecording()
{
    return FRecordingThread::isRecording();
}

bool ASimModeBase::isRecordUIVisible()
{
    return getSettings().is_record_ui_visible;
}

ECameraDirectorMode ASimModeBase::getInitialViewMode()
{
    return Utils::toEnum<ECameraDirectorMode>(getSettings().initial_view_mode);
}

void ASimModeBase::startRecording()
{
    FRecordingThread::startRecording(getFpvVehiclePawnWrapper()->getImageCapture(),
        getFpvVehiclePawnWrapper()->getTrueKinematics(), getSettings().recording_settings, getFpvVehiclePawnWrapper());
}

const AirSimSettings& ASimModeBase::getSettings() const
{
    return AirSimSettings::singleton();
}


bool ASimModeBase::toggleRecording()
{
    if (isRecording())
        stopRecording();
    else
        startRecording();

    return isRecording();
}

void ASimModeBase::stopRecording()
{
    FRecordingThread::stopRecording();
}


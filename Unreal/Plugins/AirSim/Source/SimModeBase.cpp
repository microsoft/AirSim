#include "AirSim.h"
#include "SimModeBase.h"
#include "AirBlueprintLib.h"


ASimModeBase::ASimModeBase()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();
    is_recording = false;
    record_tick_count = 0;
    setupInputBindings();
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (is_recording)
        ++record_tick_count;
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::reset()
{
    //Should be overriden by derived classes
}

std::string ASimModeBase::getReport()
{
    //Should be overriden by derived classes
    return std::string();
}

FString ASimModeBase::getReportBP()
{
    return FString(getReport().c_str());
}

void ASimModeBase::setupInputBindings()
{
    this->EnableInput(this->GetWorld()->GetFirstPlayerController());
}

bool ASimModeBase::isRecording()
{
    return is_recording;
}

void ASimModeBase::startRecording()
{
    if (record_file.is_open()) {
        record_file.close();
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was already open"), LogDebugLevel::Failure);
    }

    record_file.open(record_filename, std::ios::out | std::ios::app);

    if (record_file.is_open()) {
        is_recording = true;

        UAirBlueprintLib::LogMessage(TEXT("Recording"), TEXT("Started"), LogDebugLevel::Success);
    }
    else
        UAirBlueprintLib::LogMessage("File cannot be opened", record_filename.c_str(), LogDebugLevel::Failure);
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
    is_recording = false;
    if (!record_file.is_open()) {
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was not open"), LogDebugLevel::Failure);
    }
    else
        record_file.close();
    
    UAirBlueprintLib::LogMessage(TEXT("Recording"), TEXT("Stopped"), LogDebugLevel::Success);
}
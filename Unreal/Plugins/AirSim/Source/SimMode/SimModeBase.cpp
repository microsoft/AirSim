#include "AirSim.h"
#include "SimModeBase.h"
#include "AirBlueprintLib.h"
#include "Runtime/Launch/Resources/Version.h"

ASimModeBase::ASimModeBase()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    initializeSettings();

    is_recording = false;
    record_tick_count = 0;
    setupInputBindings();

    //check engine version
    uint16 min_major = 4, min_minor = 15;
    if ((FEngineVersion::Current().GetMajor() == min_major && FEngineVersion::Current().GetMinor() < min_minor) || 
        (FEngineVersion::Current().GetMajor() < min_major && FEngineVersion::Current().GetMajor() != 0))
        FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(TEXT(
            "Your Unreal Engine version is older and not supported."
            "If you keep running anyway, you may see some weired behaviour!\n"
            "Please upgrade to version 4.15.\n" 
            "Upgrade instructions are at https://github.com/Microsoft/AirSim/blob/master/docs/unreal_upgrade.md")));

    UAirBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""), LogDebugLevel::Informational);
}

void ASimModeBase::initializeSettings()
{
    //TODO: should this be done somewhere else?
    //load settings file if found
    typedef msr::airlib::Settings Settings;
    try {
        Settings& settings = Settings::loadJSonFile("settings.json");
        auto settings_filename = Settings::singleton().getFileName();
        std::string msg = "Loading settings from " + settings_filename;
        UAirBlueprintLib::LogMessage(FString(msg.c_str()), TEXT(""), LogDebugLevel::Informational, 30);
    }
    catch (std::exception ex) {
        UAirBlueprintLib::LogMessage(FString("Error loading settings from ~/Documents/AirSim/settings.json"), TEXT(""), LogDebugLevel::Failure, 30);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (is_recording)
        ++record_tick_count;
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::reset()
{
    //Should be overridden by derived classes
}

std::string ASimModeBase::getReport()
{
    //Should be overridden by derived classes
    return std::string();
}

FString ASimModeBase::getReportBP()
{
    return FString(getReport().c_str());
}

void ASimModeBase::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);
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

    std::string fullPath = common_utils::FileSystem::getLogFileNamePath(record_filename, "", ".txt", true);
    common_utils::FileSystem::createTextFile(fullPath, record_file);

    if (record_file.is_open()) {
        is_recording = true;

        UAirBlueprintLib::LogMessage(TEXT("Recording"), TEXT("Started"), LogDebugLevel::Success);
    }
    else
        UAirBlueprintLib::LogMessage("Error creating log file", fullPath.c_str(), LogDebugLevel::Failure);
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
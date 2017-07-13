#include "AirSim.h"
#include "SimModeBase.h"
#include "AirBlueprintLib.h"
#include "Runtime/Launch/Resources/Version.h"
#include "controllers/Settings.hpp"

ASimModeBase::ASimModeBase()
{
    PrimaryActorTick.bCanEverTick = true;
}
 
void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    recording_file_.initializeForPlay();
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

    readSettings();
}

void ASimModeBase::readSettings()
{
    typedef msr::airlib::Settings Settings;

    Settings& settings = Settings::singleton();
    enable_rpc = settings.getBool("RpcEnabled", true);
    //by default we spawn server at local endpoint. Do not use 127.0.0.1 as default below
    //because for docker container default is 0.0.0.0 and people get really confused why things
    //don't work
    api_server_address = settings.getString("LocalHostIp", "");
    is_record_ui_visible = settings.getBool("RecordUIVisible", true);

    std::string view_mode_string = settings.getString("ViewMode", "FlyWithMe");
    if (view_mode_string == "FlyWithMe")
        initial_view_mode = ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME;
    else if (view_mode_string == "Fpv")
        initial_view_mode = ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV;
    else if (view_mode_string == "Manual")
        initial_view_mode = ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL;
    else if (view_mode_string == "GroundObserver")
        initial_view_mode = ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER;

    //do not save this default in json as this will change in near future
    fpv_vehicle_name = settings.getString("FpvVehicleName", "Pixhawk");

    physics_engine_name = settings.getString("PhysicsEngineName", "FastPhysicsEngine");
    usage_scenario = settings.getString("UsageScenario", "");

    Settings record_settings;
    if (settings.getChild("Recording", record_settings)) {
        recording_settings.record_on_move = record_settings.getBool("RecordOnMove", recording_settings.record_on_move);
        recording_settings.record_interval = record_settings.getFloat("RecordInterval", recording_settings.record_interval);
    }
    
    UAirBlueprintLib::LogMessage("Vehicle name: ", fpv_vehicle_name.c_str(), LogDebugLevel::Informational);
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (recording_file_.isRecording())
        ++record_tick_count;
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::reset()
{
    //Should be overridden by derived classes
}

AVehiclePawnBase* ASimModeBase::getFpvVehiclePawn()
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
}

bool ASimModeBase::isRecording()
{
    return recording_file_.isRecording();
}

bool ASimModeBase::isRecordUIVisible()
{
    return is_record_ui_visible;
}

ECameraDirectorMode ASimModeBase::getInitialViewMode()
{
    return initial_view_mode;
}

void ASimModeBase::startRecording()
{
    recording_file_.startRecording();
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
    recording_file_.stopRecording();
}

RecordingFile& ASimModeBase::getRecordingFile()
{
    return recording_file_;
}
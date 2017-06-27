
#include "AirSim.h"
#include "AirSimGameMode.h"
#include "SimHUD/SimHUD.h"
#include "common/Common.hpp"
#include "AirBlueprintLib.h"
#include "controllers/Settings.hpp"

////for OutputDebugString
//#ifdef _MSC_VER
//#define WIN32_LEAN_AND_MEAN
//#include <windows.h>
//#endif

class AUnrealLog : public msr::airlib::Utils::Logger
{
public:
    virtual void log(int level, const std::string& message) override 
    {
        if (level == 0) {
            UE_LOG(LogAirSim, Log, TEXT("%s"), *FString(message.c_str()));
        }
        else if (level > 0) {
            UE_LOG(LogAirSim, Warning, TEXT("%s"), *FString(message.c_str()));
        }
        else {
            UE_LOG(LogAirSim, Error, TEXT("%s"), *FString(message.c_str()));
        }
    
//#ifdef _MSC_VER
//        //print to VS output window
//        OutputDebugString(std::wstring(message.begin(), message.end()).c_str());
//#endif

        //also do default logging
        msr::airlib::Utils::Logger::log(level, message);
    }
};

static AUnrealLog GlobalASimLog;

AAirSimGameMode::AAirSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    DefaultPawnClass = nullptr;
    HUDClass = ASimHUD::StaticClass();

    common_utils::Utils::getSetLogger(&GlobalASimLog);
}

void AAirSimGameMode::StartPlay() 
{
    //needs to be done before we call base class
    initializeSettings();

    Super::StartPlay();
}

void AAirSimGameMode::initializeSettings()
{
    //TODO: should this be done somewhere else?
    //load settings file if found
    typedef msr::airlib::Settings Settings;
    try {
        Settings& settings = Settings::loadJSonFile("settings.json");
        auto settings_filename = Settings::singleton().getFileName();
        if (settings.isLoadSuccess()) {
            UAirBlueprintLib::setLogMessagesHidden(! settings.getBool("LogMessagesVisible", true));

            std::string msg = "Loaded settings from " + settings_filename;
            UAirBlueprintLib::LogMessage(FString(msg.c_str()), TEXT(""), LogDebugLevel::Informational);
        }
        else {
            //write some settings in new file otherwise the string "null" is written if all settigs are empty
            settings.setString("see_docs_at", "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md");

            //settings.setBool("RpcEnabled", true);
            //settings.setString("LocalHostIp", "127.0.0.1");
            //Settings rosflight_child;
            //rosflight_child.setInt("RemoteControlID", 0);
            //settings.setChild("RosFlight", rosflight_child);

            settings.saveJSonFile(settings_filename);
            std::string msg = "Settings file " + settings_filename + " is created.";
            UAirBlueprintLib::LogMessage(FString(msg.c_str()), TEXT("See docs at https://git.io/v9mYY"), LogDebugLevel::Informational);
        }
    }
    catch (std::exception ex) {
        UAirBlueprintLib::LogMessage(FString("Error loading settings from ~/Documents/AirSim/settings.json"), TEXT(""), LogDebugLevel::Failure, 30);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}
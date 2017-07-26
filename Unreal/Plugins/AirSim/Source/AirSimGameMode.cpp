#include "AirSimGameMode.h"
#include "Misc/FileHelper.h"
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
        if (level == msr::airlib::Utils::kLogLevelError) {
            UE_LOG(LogAirSim, Error, TEXT("%s"), *FString(message.c_str()));
        }
        else if (level == msr::airlib::Utils::kLogLevelWarn) {
            UE_LOG(LogAirSim, Warning, TEXT("%s"), *FString(message.c_str()));
        }
        else {
            UE_LOG(LogAirSim, Log, TEXT("%s"), *FString(message.c_str()));
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

        /******** CAUSION: Do not use std file/IO function. They cause segfault in Linux! ******************/

        FString settings_filename = FString(Settings::getFullPath("settings.json").c_str());
        FString json_fstring;
        bool load_success = false;
        bool file_found = FPaths::FileExists(settings_filename);
        if (file_found) {
            bool read_sucess = FFileHelper::LoadFileToString(json_fstring, * settings_filename);
            if (read_sucess) {
                Settings& settings = Settings::loadJSonString(TCHAR_TO_UTF8(*json_fstring));
                if (settings.isLoadSuccess()) {
                    UAirBlueprintLib::setLogMessagesHidden(! settings.getBool("LogMessagesVisible", true));
                    UAirBlueprintLib::LogMessageString("Loaded settings from ", TCHAR_TO_UTF8(*settings_filename), LogDebugLevel::Informational);
                    load_success = true;
                }
                else
                    UAirBlueprintLib::LogMessageString("Possibly invalid json string in ", TCHAR_TO_UTF8(*settings_filename), LogDebugLevel::Failure);
            }
            else
                UAirBlueprintLib::LogMessageString("Cannot read settings from ", TCHAR_TO_UTF8(*settings_filename), LogDebugLevel::Failure);
        }

        if (!load_success) {
            //create default settings
            Settings& settings = Settings::loadJSonString("{}");
            //write some settings in new file otherwise the string "null" is written if all settigs are empty
            settings.setString("see_docs_at", "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md");

            if (!file_found) {
                std::string json_content;
//TODO: there is a crash in Linux due to settings.saveJSonString(). Remove this workaround after we know the reason.
#ifdef _WIN32
                json_content = settings.saveJSonString();
#else
                json_content = "{  \"see_docs_at\": \"https://github.com/Microsoft/AirSim/blob/master/docs/settings.md\"}";
#endif
                json_fstring = FString(json_content.c_str());
                FFileHelper::SaveStringToFile(json_fstring, * settings_filename);
                UAirBlueprintLib::LogMessageString("Created settings file at ", TCHAR_TO_UTF8(*settings_filename), LogDebugLevel::Informational);
            }
        }
    }
    catch (std::exception& ex) {
        UAirBlueprintLib::LogMessage(FString("Error loading settings from ~/Documents/AirSim/settings.json"), FString(ex.what()), LogDebugLevel::Failure, 30);
    }
}
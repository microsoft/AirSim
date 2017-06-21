
#include "AirSim.h"
#include "AirSimGameMode.h"
#include "SimHUD/SimHUD.h"
#include "common/Common.hpp"

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
    Super::StartPlay();
}
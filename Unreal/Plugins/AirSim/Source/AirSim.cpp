#include "AirSim.h"
#include "SimJoyStick/SimJoyStick.h"

DEFINE_LOG_CATEGORY(LogAirSim);

class FAirSim : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

IMPLEMENT_MODULE(FAirSim, AirSim)

void *xinput_dllHandle;

void FAirSim::StartupModule()
{
//load xinput DLL
#if defined _WIN32 || defined _WIN64
    FString filePath = *FPaths::GamePluginsDir() + FString("AirSim/Dependencies/x360ce/xinput9_1_0.dll");
    xinput_dllHandle = FPlatformProcess::GetDllHandle(*filePath); // Retrieve the DLL.
    SimJoyStick::setInitializedSuccess(xinput_dllHandle != NULL);
#endif
}

void FAirSim::ShutdownModule()
{
#if defined _WIN32 || defined _WIN64
    SimJoyStick::setInitializedSuccess(false);
    FPlatformProcess::FreeDllHandle(xinput_dllHandle);
    xinput_dllHandle = NULL;
#endif
}
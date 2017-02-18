#include "AirSim.h"

DEFINE_LOG_CATEGORY(LogAirSim);

class FAirSim : public IModuleInterface {
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

IMPLEMENT_MODULE(FAirSim, AirSim)

void FAirSim::StartupModule() {
}

void FAirSim::ShutdownModule() {
}
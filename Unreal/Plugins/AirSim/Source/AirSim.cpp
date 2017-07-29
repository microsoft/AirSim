// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY(LogAirSim);

class FAirSim : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

IMPLEMENT_MODULE(FAirSim, AirSim)

void FAirSim::StartupModule()
{
    //plugin startup
}

void FAirSim::ShutdownModule()
{
    //plugin shutdown
}
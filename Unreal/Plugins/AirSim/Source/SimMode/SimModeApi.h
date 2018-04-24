#pragma once

#include "api/SimModeApiBase.hpp"
#include "SimMode/SimModeBase.h"

class SimModeApi : msr::airlib::SimModeApiBase  {
public:
    SimModeApi(ASimModeBase* simmode);
    virtual msr::airlib::VehicleApiBase* getVehicleApi() override;

private:
    ASimModeBase* simmode_;
};
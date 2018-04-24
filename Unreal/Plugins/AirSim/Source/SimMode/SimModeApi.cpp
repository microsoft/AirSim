#include "SimModeApi.h"

SimModeApi::SimModeApi(ASimModeBase* simmode)
    : simmode_(simmode)
{
}

msr::airlib::VehicleApiBase* SimModeApi::getVehicleApi()
{
    return simmode_->getVehicleApi();
}

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_VehicleControllerBase_hpp
#define msr_air_copter_sim_VehicleControllerBase_hpp

#include "controllers/ControllerBase.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
    Defined additional interface for vehicles
*/
class VehicleControllerBase : public ControllerBase {
public:
    //tells the controller to switch from human operated mode to computer operated mode
    virtual void setOffboardMode(bool is_set) = 0;
    virtual void setSimulationMode(bool is_set) = 0;
    virtual bool isOffboardMode() = 0;
    virtual bool isSimulationMode() = 0;
    
    //Supplies the controller 16 channels of input
    virtual void setUserInputs(const vector<float>& inputs)
    {
        //default implementation
    }
};

class VehicleControllerException : public ControllerException {
public:
    VehicleControllerException(const std::string& message)
        : ControllerException(message) { 
    }
};  

class VehicleMoveException : public VehicleControllerException {
public:
    VehicleMoveException(const std::string& message)
        : VehicleControllerException(message) { 
    }
};

}} //namespace
#endif

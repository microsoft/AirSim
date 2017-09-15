// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_VehicleControllerBase_hpp
#define msr_airlib_VehicleControllerBase_hpp

#include "controllers/ControllerBase.hpp"
#include "physics/PhysicsBody.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
    Defined additional interface for vehicles
*/
class VehicleControllerBase : public ControllerBase {
public:
    //tells the controller to switch from human operated mode to computer operated mode
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual void setSimulationMode(bool is_set) = 0;
    virtual bool isApiControlEnabled() = 0;
    virtual bool isSimulationMode() = 0;

    //if controller connects via USB/UDP and connection fails then this
    //should return false
    virtual bool isAvailable(std::string& message) = 0;

    //TODO: below method is needed to support firmwares without state estimation. In future, we should probably remove this support.
    virtual void setGroundTruth(PhysicsBody* physics_body)
    {
        unused(physics_body);
        //by default don't use it. If derived class needs this, it should override.
    }
};

class VehicleControllerException : public ControllerException {
public:
    VehicleControllerException(const std::string& message)
        : ControllerException(message) { 
    }
};  

class VehicleCommandNotImplementedException : public VehicleControllerException {
public:
    VehicleCommandNotImplementedException(const std::string& message)
        : VehicleControllerException(message) { 
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

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "common/Waiter.hpp"
#include "safety/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
Vehicle controller allows to obtain state from vehicle and send control commands to the vehicle.
State can include many things including sensor data, logs, estimated state from onboard computer etc.
Control commands can be low level actuation commands or high level movement commands.
The base class defines usually available methods that all vehicle controllers may implement.
Some methods may not be applicable to specific vehicle in which case an exception may be raised or call may be ignored.
*/
class VehicleApiBase : public UpdatableObject {
public:
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() const = 0;
    virtual bool armDisarm(bool arm) = 0;
    virtual GeoPoint getHomeGeoPoint() const = 0;

    //default implementation so derived class doesn't have to call on UpdatableObject
    virtual void reset() override
    {
        UpdatableObject::reset();
    }
    virtual void update() override
    {
        UpdatableObject::update();
    }

    virtual void cancelLastTask()
    {
        //if derived class supports async task then override this method
    }
    virtual bool isReady(std::string& message) const
    {
        unused(message);
        return true;
    }

    //if vehicle supports it, call this method to send
    //kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
    virtual void sendTelemetry(float last_interval = -1)
    {
        //no default action
        unused(last_interval);
    }

    //below APIs are used by FastPhysicsEngine
    virtual real_T getActuation(unsigned int actuator_index) const
    {
        unused(actuator_index);
        throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
    }
    virtual size_t getActuatorCount() const
    {
        throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
    }

    virtual void getStatusMessages(std::vector<std::string>& messages)
    {
        unused(messages);
        //default implementation
    }

    /*
    For RCs, there are two cases: (1) vehicle may be configured to use
    RC bound to its hardware (2) vehicle may be configured to get RC data
    supplied via API calls. Below two APIs are not symmetrical, i.e.,
    getRCData() may or may not return same thing as setRCData().
    */
    //get reading from RC bound to vehicle (if unsupported then RCData::is_valid = false)
    virtual RCData getRCData() const
    {
        static const RCData invalid_rc_data;
        return invalid_rc_data;
    }
    //set external RC data to vehicle (if unsupported then returns false)
    virtual bool setRCData(const RCData& rc_data)
    {
        unused(rc_data);
        return false;
    }

    virtual ~VehicleApiBase() = default;

    //exceptions
    class VehicleControllerException : public std::runtime_error {
    public:
        VehicleControllerException(const std::string& message)
            : runtime_error(message) {
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
};


}} //namespace
#endif

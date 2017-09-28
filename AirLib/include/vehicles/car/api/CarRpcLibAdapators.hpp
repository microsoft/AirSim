// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibAdapators_hpp
#define air_CarRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "controllers/VehicleCameraBase.hpp"
#include "vehicles/car/controllers/CarControllerBase.hpp"
#include "rpc/msgpack.hpp"


namespace msr { namespace airlib_rpclib {

class CarRpcLibAdapators : public RpcLibAdapatorsBase {
public:
    struct CarControls {
        float throttle = 0;
        float steering = 0;
        bool handbreak = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = false;

        MSGPACK_DEFINE_MAP(throttle, steering, handbreak, is_manual_gear, manual_gear, gear_immediate);

        CarControls()
        {}

        CarControls(const msr::airlib::CarControllerBase::CarControls& s)
        {
            throttle = s.throttle;
            steering = s.steering;
            handbreak = s.handbreak;
            is_manual_gear = s.is_manual_gear;
            manual_gear = s.manual_gear;
            gear_immediate = s.gear_immediate;
        }
        msr::airlib::CarControllerBase::CarControls to() const
        {
            return msr::airlib::CarControllerBase::CarControls(throttle, steering, handbreak, 
                is_manual_gear, manual_gear, gear_immediate);
        }
    };

    struct CarState {
        float speed;
        int gear;
        Vector3r position;
        Vector3r velocity;
        Quaternionr orientation;

        MSGPACK_DEFINE_MAP(speed, gear, position, velocity, orientation);

        CarState()
        {}

        CarState(const msr::airlib::CarControllerBase::CarState& s)
        {
            speed = s.speed;
            gear = s.gear;
            position = s.position;
            velocity = s.velocity;
            orientation = s.orientation;
        }
        msr::airlib::CarControllerBase::CarState to() const
        {
            return msr::airlib::CarControllerBase::CarState(
                speed, gear, position.to(), velocity.to(), orientation.to());
        }
    };
};

}} //namespace


#endif

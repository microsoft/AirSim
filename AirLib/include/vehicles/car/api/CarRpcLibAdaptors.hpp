// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibAdaptors_hpp
#define air_CarRpcLibAdaptors_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr { namespace airlib_rpclib {

class CarRpcLibAdaptors : public RpcLibAdaptorsBase {
public:
    struct CarControls {
        float throttle = 0;
        float steering = 0;
        float brake = 0;
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        MSGPACK_DEFINE_MAP(throttle, steering, brake, handbrake, is_manual_gear, manual_gear, gear_immediate);

        CarControls()
        {}

        CarControls(const msr::airlib::CarApiBase::CarControls& s)
        {
            throttle = s.throttle;
            steering = s.steering;
            brake = s.brake;
            handbrake = s.handbrake;
            is_manual_gear = s.is_manual_gear;
            manual_gear = s.manual_gear;
            gear_immediate = s.gear_immediate;
        }
        msr::airlib::CarApiBase::CarControls to() const
        {
            return msr::airlib::CarApiBase::CarControls(throttle, steering, brake, handbrake,
                is_manual_gear, manual_gear, gear_immediate);
        }
    };

    struct CarState {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        KinematicsState kinematics_estimated;
        uint64_t timestamp;

        MSGPACK_DEFINE_MAP(speed, gear, rpm, maxrpm, handbrake, kinematics_estimated, timestamp);

        CarState()
        {}

        CarState(const msr::airlib::CarApiBase::CarState& s)
        {
            speed = s.speed;
            gear = s.gear;
            rpm = s.rpm;
            maxrpm = s.maxrpm;
            handbrake = s.handbrake;
            timestamp = s.timestamp;
            kinematics_estimated = s.kinematics_estimated;
        }
        msr::airlib::CarApiBase::CarState to() const
        {
            return msr::airlib::CarApiBase::CarState(
                speed, gear, rpm, maxrpm, handbrake, kinematics_estimated.to(), timestamp);
        }
    };
};

}} //namespace


#endif

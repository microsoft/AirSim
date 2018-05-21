// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibAdapators_hpp
#define air_MultirotorRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "vehicles/multirotor/controllers/MultirotorCommon.hpp"
#include "vehicles/multirotor/controllers/MultirotorApiBase.h"
#include "common/ImageCaptureBase.hpp"
#include "safety/SafetyEval.hpp"
#include "rpc/msgpack.hpp"


namespace msr { namespace airlib_rpclib {

class MultirotorRpcLibAdapators : public RpcLibAdapatorsBase {
public:
    struct YawMode {
        bool is_rate = true;
        float yaw_or_rate = 0;
        MSGPACK_DEFINE_MAP(is_rate, yaw_or_rate);
    
        YawMode()
        {}

        YawMode(const msr::airlib::YawMode& s)
        {
            is_rate = s.is_rate;
            yaw_or_rate = s.yaw_or_rate;
        }
        msr::airlib::YawMode to() const
        {
            return msr::airlib::YawMode(is_rate, yaw_or_rate);
        }
    };

    struct MultirotorState {
        CollisionInfo collision;
        KinematicsState kinematics_estimated;
        KinematicsState kinematics_true;
        GeoPoint gps_location;
        uint64_t timestamp;
        LandedState landed_state;
        RCData rc_data;
        std::vector<std::string> controller_messages;


        MSGPACK_DEFINE_MAP(collision, kinematics_estimated, kinematics_true, gps_location, timestamp, landed_state, rc_data, controller_messages);

        MultirotorState()
        {}

        MultirotorState(const msr::airlib::MultirotorState& s)
        {
            collision = s.collision;
            kinematics_estimated = s.kinematics_estimated;
            kinematics_true = s.kinematics_true;
            gps_location = s.gps_location;
            timestamp = s.timestamp;
            landed_state = s.landed_state;
            rc_data = RCData(s.rc_data);
            controller_messages = s.controller_messages;
        }

        msr::airlib::MultirotorState to() const
        {
            return msr::airlib::MultirotorState(collision.to(), kinematics_estimated.to(), 
                kinematics_true.to(), gps_location.to(), timestamp, landed_state, rc_data.to(), controller_messages);
        }
    };
};

}} //namespace

MSGPACK_ADD_ENUM(msr::airlib::DrivetrainType);
MSGPACK_ADD_ENUM(msr::airlib::LandedState);


#endif

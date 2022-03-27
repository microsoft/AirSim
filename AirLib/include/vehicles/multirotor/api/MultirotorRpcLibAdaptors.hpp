// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibAdaptors_hpp
#define air_MultirotorRpcLibAdaptors_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "safety/SafetyEval.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr
{
namespace airlib_rpclib
{

    class MultirotorRpcLibAdaptors : public RpcLibAdaptorsBase
    {
    public:
        struct YawMode
        {
            bool is_rate = true;
            float yaw_or_rate = 0;
            MSGPACK_DEFINE_MAP(is_rate, yaw_or_rate);

            YawMode()
            {
            }

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

        struct RotorParameters
        {
            msr::airlib::real_T thrust;
            msr::airlib::real_T torque_scaler;
            msr::airlib::real_T speed;

            MSGPACK_DEFINE_MAP(thrust, torque_scaler, speed);

            RotorParameters()
            {
            }

            RotorParameters(const msr::airlib::RotorParameters& s)
            {
                thrust = s.thrust;
                torque_scaler = s.torque_scaler;
                speed = s.speed;
            }

            msr::airlib::RotorParameters to() const
            {
                return msr::airlib::RotorParameters(thrust, torque_scaler, speed);
            }
        };

        struct RotorStates
        {
            std::vector<RotorParameters> rotors;
            uint64_t timestamp;

            MSGPACK_DEFINE_MAP(rotors, timestamp);

            RotorStates()
            {
            }

            RotorStates(const msr::airlib::RotorStates& s)
            {
                for (const auto& r : s.rotors) {
                    rotors.push_back(RotorParameters(r));
                }
                timestamp = s.timestamp;
            }

            msr::airlib::RotorStates to() const
            {
                std::vector<msr::airlib::RotorParameters> d;
                for (const auto& r : rotors) {
                    d.push_back(r.to());
                }
                return msr::airlib::RotorStates(d, timestamp);
            }
        };

        struct MultirotorState
        {
            CollisionInfo collision;
            KinematicsState kinematics_estimated;
            KinematicsState kinematics_true;
            GeoPoint gps_location;
            uint64_t timestamp;
            msr::airlib::LandedState landed_state;
            RCData rc_data;
            bool ready;
            std::string ready_message;
            std::vector<std::string> controller_messages;
            bool can_arm;

            MSGPACK_DEFINE_MAP(collision, kinematics_estimated, gps_location, timestamp, landed_state, rc_data);

            MultirotorState()
            {
            }

            MultirotorState(const msr::airlib::MultirotorState& s)
            {
                collision = s.collision;
                kinematics_estimated = s.kinematics_estimated;
                gps_location = s.gps_location;
                timestamp = s.timestamp;
                landed_state = s.landed_state;
                rc_data = RCData(s.rc_data);
                ready = s.ready;
                ready_message = s.ready_message;
                can_arm = s.can_arm;
            }

            msr::airlib::MultirotorState to() const
            {
                return msr::airlib::MultirotorState(collision.to(), kinematics_estimated.to(), gps_location.to(), timestamp, landed_state, rc_data.to(), ready, ready_message, can_arm);
            }
        };
    };
}
} //namespace

MSGPACK_ADD_ENUM(msr::airlib::DrivetrainType);
MSGPACK_ADD_ENUM(msr::airlib::LandedState);

#endif

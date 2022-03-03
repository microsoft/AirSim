#ifndef air_WarthogRpcLibAdapters_hpp
#define air_WarthogRpcLibAdapters_hpp
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/warthog/api/WarthogApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr
{
namespace airlib_rpclib
{

    class WarthogRpcLibAdaptors : public RpcLibAdaptorsBase
    {
    public:
        struct WarthogControls
        {
            //float throttle = 0;
            //float steering = 0;
            //float brake = 0;
            //bool handbrake = false;
            //bool is_manual_gear = false;
            //int manual_gear = 0;
            //bool gear_immediate = true;
            float linear_vel = 0;
            float angular_vel = 0;

           // MSGPACK_DEFINE_MAP(throttle, steering, brake, handbrake, is_manual_gear, manual_gear, gear_immediate);
            MSGPACK_DEFINE_MAP(linear_vel, angular_vel);

            WarthogControls()
            {
            }

            WarthogControls(const msr::airlib::WarthogApiBase::WarthogControls& s)
            {
                linear_vel = s.linear_vel;
                angular_vel = s.angular_vel;
            }
            msr::airlib::WarthogApiBase::WarthogControls to() const
            {
                return msr::airlib::WarthogApiBase::WarthogControls(linear_vel, angular_vel);
            }
        };

        struct WarthogState
        {
            //float speed;
            //int gear;
            //float rpm;
            //float maxrpm;
            //bool handbrake;
            KinematicsState kinematics_estimated;
            float linear_vel;
            float angular_vel;
            //uint64_t timestamp;

            //MSGPACK_DEFINE_MAP(speed, gear, rpm, maxrpm, handbrake, kinematics_estimated, timestamp);
            MSGPACK_DEFINE_MAP(linear_vel, angular_vel, kinematics_estimated);

            WarthogState()
            {
            }

            
            WarthogState(const msr::airlib::WarthogApiBase::WarthogState& s)
            {
                linear_vel = s.linear_vel;
                angular_vel = s.angular_vel;
                kinematics_estimated = s.kinematics_estimated;
            }
            msr::airlib::WarthogApiBase::WarthogState to() const
            {
                return msr::airlib::WarthogApiBase::WarthogState(
                    linear_vel, angular_vel, kinematics_estimated.to());
            }
        };
    };
}
} //namespace

#endif
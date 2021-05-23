#ifndef air_DroneCommon_hpp
#define air_DroneCommon_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"

namespace msr
{
namespace airlib
{

    enum class DrivetrainType
    {
        MaxDegreeOfFreedom = 0,
        ForwardOnly
    };

    enum class LandedState : uint
    {
        Landed = 0,
        Flying = 1
    };
    // Structs for rotor state API
    struct RotorParameters
    {
        real_T thrust = 0;
        real_T torque_scaler = 0;
        real_T speed = 0;

        RotorParameters()
        {
        }

        RotorParameters(const real_T& thrust_val, const real_T& torque_scaler_val, const real_T& speed_val)
            : thrust(thrust_val), torque_scaler(torque_scaler_val), speed(speed_val)
        {
        }

        void update(const real_T& thrust_val, const real_T& torque_scaler_val, const real_T& speed_val)
        {
            thrust = thrust_val;
            torque_scaler = torque_scaler_val;
            speed = speed_val;
        }
    };

    struct RotorStates
    {
        std::vector<RotorParameters> rotors;
        uint64_t timestamp;

        RotorStates()
        {
        }
        RotorStates(const std::vector<RotorParameters>& rotors_val, uint64_t timestamp_val)
            : rotors(rotors_val), timestamp(timestamp_val)
        {
        }
    };

    //Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone
    struct YawMode
    {
        bool is_rate = true;
        float yaw_or_rate = 0.0f;

        YawMode()
        {
        }

        YawMode(bool is_rate_val, float yaw_or_rate_val)
        {
            is_rate = is_rate_val;
            yaw_or_rate = yaw_or_rate_val;
        }

        static YawMode Zero()
        {
            return YawMode(true, 0);
        }

        void setZeroRate()
        {
            is_rate = true;
            yaw_or_rate = 0;
        }
    };

    //properties of vehicle
    struct MultirotorApiParams
    {
        MultirotorApiParams(){};
        //what is the breaking distance for given velocity?
        //Below is just proportionality constant to convert from velocity to breaking distance
        float vel_to_breaking_dist = 0.5f; //ideally this should be 2X for very high speed but for testing we are keeping it 0.5
        float min_breaking_dist = 1; //min breaking distance
        float max_breaking_dist = 3; //min breaking distance
        float breaking_vel = 1.0f;
        float min_vel_for_breaking = 3;

        //what is the differential positional accuracy of cur_loc?
        //this is not same as GPS accuracy because translational errors
        //usually cancel out. Typically this would be 0.2m or less
        float distance_accuracy = 0.1f;

        //what is the minimum clearance from obstacles?
        float obs_clearance = 2;

        //what is the +/-window we should check on obstacle map?
        //for example 2 means check from ticks -2 to 2
        int obs_window = 0;
    };

    struct MultirotorState
    {
        CollisionInfo collision;
        Kinematics::State kinematics_estimated;
        GeoPoint gps_location;
        uint64_t timestamp;
        LandedState landed_state;
        RCData rc_data;
        bool ready; // indicates drone is ready for commands
        std::string ready_message; // can show error message if drone is not reachable over the network or is not responding
        bool can_arm; // indicates drone is ready to be armed

        MultirotorState()
        {
        }
        MultirotorState(const CollisionInfo& collision_val, const Kinematics::State& kinematics_estimated_val,
                        const GeoPoint& gps_location_val, uint64_t timestamp_val,
                        LandedState landed_state_val, const RCData& rc_data_val, bool ready_val, const std::string& message, bool can_arm_val)
            : collision(collision_val), kinematics_estimated(kinematics_estimated_val), gps_location(gps_location_val), timestamp(timestamp_val), landed_state(landed_state_val), rc_data(rc_data_val), ready(ready_val), ready_message(message), can_arm(can_arm_val)
        {
        }

        //shortcuts
        const Vector3r& getPosition() const
        {
            return kinematics_estimated.pose.position;
        }
        const Quaternionr& getOrientation() const
        {
            return kinematics_estimated.pose.orientation;
        }
    };
}
} //namespace
#endif
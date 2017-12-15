#ifndef air_DroneCommon_hpp
#define air_DroneCommon_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"

namespace msr { namespace airlib {


enum class DrivetrainType {
    MaxDegreeOfFreedom = 0,
    ForwardOnly
};

//Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone
struct YawMode {
    bool is_rate = true;
    float yaw_or_rate = 0.0f;

    YawMode()
    {}

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
struct VehicleParams {
    VehicleParams(){}; 
    //what is the breaking distance for given velocity?
    //Below is just proportionalty constant to convert from velocity to breaking distance
    float vel_to_breaking_dist = 0.5f;   //ideally this should be 2X for very high speed but for testing we are keeping it 0.5
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

struct RCData {
    TTimePoint timestamp = 0;
    //pitch, roll, yaw should be in range -1 to 1
    //switches should be integer value indicating its state, 0=on, 1=off for example.
    float pitch = 0, roll = 0, throttle = 0, yaw = 0;
    unsigned int  switch1 = 0, switch2 = 0, switch3 = 0, switch4 = 0, 
        switch5 = 0, switch6 = 0, switch7 = 0, switch8 = 0;
    bool is_initialized = false; //is RC connected?
    bool is_valid = false; //must be true for data to be valid


    void add(const RCData& other)
    {
        pitch += other.pitch; roll += other.roll; throttle += other.throttle; yaw += other.yaw;
    }
    void subtract(const RCData& other)
    {
        pitch -= other.pitch; roll -= other.roll; throttle -= other.throttle; yaw -= other.yaw;
    }
    void divideBy(float k)
    {
        pitch /= k; roll /= k; throttle /= k; yaw /= k;   
    }
    bool isAnyMoreThan(float k)
    {
        using std::abs;
        return abs(pitch) > k || abs(roll) > k || abs(throttle) > k || abs(yaw) > k;
    }
    string toString()
    {
        return Utils::stringf("RCData[pitch=%f, roll=%f, throttle=%f, yaw=%f]", pitch, roll, throttle, yaw);
    }
};

struct MultirotorState {
    CollisionInfo collision;
    Kinematics::State kinematics_estimated;
    Kinematics::State kinematics_true;
    GeoPoint gps_location;
    uint64_t timestamp;

    MultirotorState()
    {}
    MultirotorState(const CollisionInfo& collision_val, const Kinematics::State& kinematics_true_val, 
        const Kinematics::State& kinematics_estimated_val, const GeoPoint& gps_location_val, uint64_t timestamp_val)
        : collision(collision_val), kinematics_estimated(kinematics_estimated_val), 
        kinematics_true(kinematics_true_val), gps_location(gps_location_val), timestamp(timestamp_val)
    {
    }
};

}} //namespace

#endif
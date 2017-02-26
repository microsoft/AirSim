#ifndef air_DroneCommon_hpp
#define air_DroneCommon_hpp

#include "common/Common.hpp"


namespace msr { namespace airlib {


enum class DrivetrainType {
    MaxDegreeOfFreedome = 0,
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
    //what is the breaking distance for given velocity?
    //currently we support simple linear relationship
    float vel_to_breaking_dist = 0.5f;   //ideally this should be 2X for very high speed but for testing we are keeping it 0.5
    float min_vel_to_breaking_dist = 1;
    float breaking_vel = 0.25f;

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
    double timestamp;
    float pitch = 0, roll = 0, throttle = 0, yaw = 0, switch1 = 0, switch2 = 0, switch3 = 0;

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

}} //namespace

#endif
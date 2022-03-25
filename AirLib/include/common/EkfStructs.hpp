
#ifndef msr_airlib_EkfStructs_hpp
#define msr_airlib_EkfStructs_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

struct SensorMeasurements
{
    Vector3r accel;
    Vector3r gyro;

    Vector3r gps_position;
    Vector3r gps_velocity;

    real_T baro_altitude;

    Vector3r magnetic_flux;
};

struct SensorBiases
{
    Vector3r accel;
    Vector3r gyro;

    real_T barometer;
};

struct EkfKinematicsState
{
    Vector3r position;
    Quaternionr orientation;
    Vector3r angles;

    Vector3r linear_velocity;

    SensorBiases sensor_bias;
};

}
} //namespace
#endif
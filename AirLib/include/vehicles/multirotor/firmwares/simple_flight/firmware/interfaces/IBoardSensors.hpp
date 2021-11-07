#pragma once

namespace simple_flight
{

class IBoardSensors
{
public:
    virtual void readAccel(float accel[3]) const = 0; //accel in m/s^2
    virtual void readGyro(float gyro[3]) const = 0; //angular velocity vector rad/sec

    // // added by Suman
    // virtual void readImuData(float accel[3], float gyro[3]) const = 0;
    // virtual void readBarometerData(float* altitude) const = 0;
    // virtual void readMagnetometerData(float mag[3]) const = 0;
    // virtual void readGpsData(float geo[3], float vel[3]) const = 0;
};
}
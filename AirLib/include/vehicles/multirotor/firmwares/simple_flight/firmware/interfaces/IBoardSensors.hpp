#pragma once

namespace simple_flight
{

class IBoardSensors
{
public:
    virtual void readAccel(float accel[3]) const = 0; //accel in m/s^2
    virtual void readGyro(float gyro[3]) const = 0; //angular velocity vector rad/sec

    virtual bool checkImuIfNew() const = 0;
    virtual bool checkBarometerIfNew() const = 0;
    virtual bool checkMagnetometerIfNew() const = 0;
    virtual bool checkGpsIfNew() const = 0;
    virtual void readImuData(real_T accel[3], real_T gyro[3]) const = 0;
    virtual void readBarometerData(real_T* altitude) const = 0;
    virtual void readMagnetometerData(real_T mag[3]) const = 0;
    virtual void readGpsData(double geo[3], real_T vel[3]) const = 0;
};
}
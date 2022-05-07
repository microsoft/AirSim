#pragma once

namespace simple_flight
{

class IBoardSensors
{
public:
    virtual void readAccel(float accel[3]) const = 0; //accel in m/s^2
    virtual void readGyro(float gyro[3]) const = 0; //angular velocity vector rad/sec

    virtual ~IBoardSensors() = default;
};
}

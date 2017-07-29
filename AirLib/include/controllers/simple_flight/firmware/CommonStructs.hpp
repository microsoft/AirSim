#pragma once

namespace simple_flight {


struct ControlMode {
    bool PitchByRate = false;
    bool RollByRate = false;
    bool YawByRate = true;

    static ControlMode getStandardAngleMode()
    {
        return ControlMode();
    }

    static ControlMode getAllRateMode()
    {
        ControlMode c;
        c.PitchByRate = true;
        c.RollByRate = true;
        c.YawByRate = true;
    }

};

struct Angles {
    float pitch = 0, roll = 0, yaw = 0;
};

struct Controls {
    float throttle = 0;
    Angles angles;
};

class IBoardClock {
public:
    virtual uint64_t micros() const = 0;
    virtual uint64_t millis() const = 0;
};

class IBoardInputPins {
public:
    virtual float readChannel(uint8_t index) const = 0; //output -1 to 1
};

class IBoardOutputPins {
public:
    virtual void writeOutput(uint8_t index, float val) = 0; //val = -1 to 1 for reversible motors otherwise 0 to 1
    virtual void setLed(uint8_t index, int32_t color) = 0;
};

class IBoardSensors {
public:
    virtual void readAccel(float accel[3]) const = 0; //accel in m/s^2
    virtual void readGyro(float gyro[3]) const = 0; //angular velocity vector rad/sec
};

class IAngleEstimator {
public:
    virtual Angles getAngles() const = 0;
};

} //namespace
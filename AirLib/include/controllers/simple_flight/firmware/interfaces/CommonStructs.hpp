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

        return c;
    }
};

struct Angles {
    float pitch = 0, roll = 0, yaw = 0;
};

struct Controls {
    float throttle = 0;
    Angles angles;
};

} //namespace
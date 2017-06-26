#pragma once

namespace simple_flight {

struct Controls {
    float throttle = 0, pitch = 0, roll = 0, yaw = 0;
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

} //namespace
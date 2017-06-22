#pragma once

#include <cstdint>

namespace simple_flight {

class Board {
public:
    virtual uint64_t micros() const = 0;
    virtual uint64_t millis() const = 0;
    virtual uint16_t readChannel(int16_t channel) const = 0;
    virtual void writeOutput(uint8_t index, uint16_t pwm) = 0;
    virtual void setLed(uint8_t index, int32_t color) = 0;
    virtual void readAccel(float accel[3]) const = 0;
    virtual void readGyro(float gyro[3]) const = 0;
    virtual void delayMicros(uint32_t us) = 0;
    virtual void delayMillis(uint32_t ms) = 0;
    virtual void reset() = 0;
    virtual void update() = 0;
};


} //namespace
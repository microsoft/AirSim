#pragma once

#include <cstdint>
#include <functional>


namespace ros_flight {

class Board {
public: //types
    enum SensorType {
        Imu = 0,
        Baro = 1,
        Mag = 2,
        Gps = 3,
        Sonar = 4,
        DiffPressure = 5
    };

public:
    virtual void init() = 0;
    virtual uint64_t micros() = 0;
    virtual uint32_t millis() = 0;
    virtual void init_sensors(uint16_t& acc1G, float& gyro_scale, int boardVersion, const std::function<void(void)>& imu_updated_callback) = 0;
    virtual bool is_sensor_present(SensorType type) = 0;
    virtual uint16_t pwmRead(int16_t channel) = 0;
    virtual void pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec) = 0;
    virtual void pwmWriteMotor(uint8_t index, uint16_t value) = 0;
    virtual void set_led(uint8_t index, bool is_on) = 0;
    virtual void toggle_led(uint8_t index) = 0;
    virtual void init_params() = 0;
    virtual bool read_params() = 0;
    virtual bool write_params(bool blink_led) = 0;
    virtual void init_imu(uint16_t& acc1G, float& gyroScale, int boardVersion) = 0;
    virtual void read_accel(int16_t accel_adc[3]) = 0;
    virtual void read_gyro(int16_t gyro_adc[3]) = 0;
    virtual void read_temperature(int16_t& temp) = 0;
    virtual void read_baro(float& altitude, float& pressure, float& temperature) = 0;
    virtual void read_diff_pressure(float& differential_pressure, float& temp, float& velocity) = 0;
    virtual float read_sonar() = 0;
    virtual void read_mag(int16_t mag_adc[3]) = 0;
    virtual void delay_micros(uint32_t us) = 0;
    virtual void delay_millis(uint32_t ms) = 0;
    virtual void system_reset(bool toBootloader) = 0;
};


} //namespace
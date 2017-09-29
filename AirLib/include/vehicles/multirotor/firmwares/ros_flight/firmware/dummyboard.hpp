#pragma once

#include "board.hpp"


namespace ros_flight {

class DummyBoard : public Board {
public:
    virtual void init() override {}
    virtual uint64_t micros() override { return 0; }
    virtual uint32_t millis() override { return 0; }
    virtual void init_sensors(uint16_t& acc1G, float& gyro_scale, int boardVersion, const std::function<void(void)>& imu_updated_callback) override {}
    virtual bool is_sensor_present(SensorType type) override { return false; }
    virtual uint16_t pwmRead(int16_t channel) override { return 0; }
    virtual void pwmInit(bool useCPPM, bool usePwmFilter, bool fastPWM, uint32_t motorPwmRate, uint16_t idlePulseUsec) override {}
    virtual void pwmWriteMotor(uint8_t index, uint16_t value) override {}
    virtual void set_led(uint8_t index, bool is_on) override {}
    virtual void toggle_led(uint8_t index) override {}
    virtual void init_params() override {}
    virtual bool read_params() override { return false;  }
    virtual bool write_params(bool blink_led) override { return false; }
    virtual void init_imu(uint16_t& acc1G, float& gyroScale, int boardVersion) override {}
    virtual void read_accel(int16_t accel_adc[3]) override {}
    virtual void read_gyro(int16_t gyro_adc[3]) override {}
    virtual void read_temperature(int16_t& temp) override {}
    virtual void read_baro(float& altitude, float& pressure, float& temperature) override {}
    virtual void read_diff_pressure(float& differential_pressure, float& temp, float& velocity) override {}
    virtual float read_sonar() override { return 0; }
    virtual void read_mag(int16_t mag_adc[3]) override {}
    virtual void delay_micros(uint32_t us) override {}
    virtual void delay_millis(uint32_t ms) override {}
    virtual void system_reset(bool toBootloader) {}
};


} //namespace
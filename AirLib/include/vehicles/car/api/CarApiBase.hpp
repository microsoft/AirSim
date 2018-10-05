// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr { namespace airlib {

class CarApiBase : public VehicleApiBase  {
public:
    struct CarControls {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
            bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val),
            is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
        }
        void set_throttle(float throttle_val, bool forward)
        {
            if (forward) {
                is_manual_gear = false;
                manual_gear = 0;
                throttle = std::abs(throttle_val);
            }
            else {
                is_manual_gear = false;
                manual_gear = -1;
                throttle = - std::abs(throttle_val);
            }
        }
    };

    struct CarState {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        Kinematics::State kinematics_estimated;
        uint64_t timestamp;

        CarState(float speed_val, int gear_val, float rpm_val, float maxrpm_val, bool handbrake_val, 
            const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
            : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), 
              kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
        {
        }
    };

    // sensors for cars
    // TODO: Some of this sensor code is duplicate with multi-rotor sensor work.
    // But there are subtle structural differences like lack of equivalent MultiRotor 
    // physics body and associated params. So keeping this code here for now and 
    // avoiding refactoring for now.
    struct EnabledSensors {
        bool imu = false;
        bool magnetometer = false;
        bool gps = false;
        bool barometer = false;
        bool distance = false;  //this causes ray casts so disabled by default
        bool lidar = false;     //this causes ray casts so disabled by default; lidar_setting
    };

public:
    CarApiBase(std::shared_ptr<msr::airlib::SensorFactory> sensor_factory, const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment)
    {
        initialize(sensor_factory, state, environment);
    }

    //default implementation so derived class doesn't have to call on VehicleApiBase
    virtual void reset() override
    {
        VehicleApiBase::reset();

        //reset sensors last after their ground truth has been reset
        getSensors().reset();
    }
    virtual void update() override
    {
        VehicleApiBase::update();

        getSensors().update();
    }
    void reportState(StateReporter& reporter)
    {
        getSensors().reportState(reporter);
    }

    // sensor helpers
    virtual const SensorCollection& getSensors() const override
    {
        return sensors_;
    }

    SensorCollection& getSensors()
    {
        return sensors_;
    }

    void initialize(std::shared_ptr<SensorFactory> sensor_factory, const Kinematics::State& state, const Environment& environment)
    {
        sensor_factory_ = sensor_factory;

        sensor_storage_.clear();
        sensors_.clear();

        EnabledSensors enabledSensors;
        addEnabledSensors(enabledSensors);

        getSensors().initialize(&state, &environment);
    }

    void addEnabledSensors(const EnabledSensors& enabled_sensors)
    {
        if (enabled_sensors.imu)
            addSensor(SensorBase::SensorType::Imu);
        if (enabled_sensors.magnetometer)
            addSensor(SensorBase::SensorType::Magnetometer);
        if (enabled_sensors.gps)
            addSensor(SensorBase::SensorType::Gps);
        if (enabled_sensors.barometer)
            addSensor(SensorBase::SensorType::Barometer);
        if (enabled_sensors.distance)
            addSensor(SensorBase::SensorType::Distance);
        if (enabled_sensors.lidar)
            addSensor(SensorBase::SensorType::Lidar);
    }

    SensorBase* addSensor(SensorBase::SensorType sensor_type)
    {
        std::unique_ptr<SensorBase> sensor = createSensor(sensor_type);
        if (sensor) {
            SensorBase* sensor_temp = sensor.get();
            sensor_storage_.push_back(std::move(sensor));
            sensors_.insert(sensor_temp, sensor_type);
            return sensor_temp;
        }
        return nullptr;
    }

    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type)
    {
        return sensor_factory_->createSensor(sensor_type);
    }

    virtual void setCarControls(const CarControls& controls) = 0;
    virtual CarState getCarState() const = 0;
    virtual const CarApiBase::CarControls& getCarControls() const = 0;

    virtual ~CarApiBase() = default;

    std::shared_ptr<const SensorFactory> sensor_factory_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
};


}} //namespace
#endif
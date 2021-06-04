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

namespace msr
{
namespace airlib
{

    class CarApiBase : public VehicleApiBase
    {
    public:
        struct CarControls
        {
            float throttle = 0; /* 1 to -1 */
            float steering = 0; /* 1 to -1 */
            float brake = 0; /* 1 to -1 */
            bool handbrake = false;
            bool is_manual_gear = false;
            int manual_gear = 0;
            bool gear_immediate = true;

            CarControls()
            {
            }
            CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
                        bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
                : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val), is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
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
                    throttle = -std::abs(throttle_val);
                }
            }
        };

        struct CarState
        {
            float speed;
            int gear;
            float rpm;
            float maxrpm;
            bool handbrake;
            Kinematics::State kinematics_estimated;
            uint64_t timestamp;

            CarState()
            {
            }

            CarState(float speed_val, int gear_val, float rpm_val, float maxrpm_val, bool handbrake_val,
                     const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
                : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const Vector3r& getPosition() const
            {
                return kinematics_estimated.pose.position;
            }
            const Quaternionr& getOrientation() const
            {
                return kinematics_estimated.pose.orientation;
            }
        };

    public:
        // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
        //CarApiBase() {}

        CarApiBase(const AirSimSettings::VehicleSetting* vehicle_setting,
                   std::shared_ptr<SensorFactory> sensor_factory,
                   const Kinematics::State& state, const Environment& environment)
        {
            initialize(vehicle_setting, sensor_factory, state, environment);
        }

        virtual void update() override
        {
            VehicleApiBase::update();

            getSensors().update();
        }

        void reportState(StateReporter& reporter) override
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

        void initialize(const AirSimSettings::VehicleSetting* vehicle_setting,
                        std::shared_ptr<SensorFactory> sensor_factory,
                        const Kinematics::State& state, const Environment& environment)
        {
            sensor_factory_ = sensor_factory;

            sensor_storage_.clear();
            sensors_.clear();

            addSensorsFromSettings(vehicle_setting);

            getSensors().initialize(&state, &environment);
        }

        void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            const auto& sensor_settings = vehicle_setting->sensors;

            sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
        }

        virtual void setCarControls(const CarControls& controls) = 0;
        virtual void updateCarState(const CarState& state) = 0;
        virtual const CarState& getCarState() const = 0;
        virtual const CarControls& getCarControls() const = 0;

        virtual ~CarApiBase() = default;

        std::shared_ptr<const SensorFactory> sensor_factory_;
        SensorCollection sensors_; //maintains sensor type indexed collection of sensors
        vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors

    protected:
        virtual void resetImplementation() override
        {
            //reset sensors last after their ground truth has been reset
            getSensors().reset();
        }
    };
}
} //namespace
#endif

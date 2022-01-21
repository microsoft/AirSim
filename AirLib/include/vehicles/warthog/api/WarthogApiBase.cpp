#ifndef air_WarthogApiBase_hpp
#define air_WarthogApiBase_hpp

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
    class WarthogApiBase : public VehicleApiBase
    {
    public:
        struct WarthogControls
        {
            float linear_vel = 0; // -5 to 5
            float angular_vel = 0; // -2 to 2
            WarthogControls()
            {
            }
            WarthogControls(float linear_vel, float angular_vel)
                : linear_vel(linear_vel), angular_vel(angular_vel)
            {
            }
            void set_linear_vel(float linear_vel_input)
            {
                linear_vel = linear_vel_input;
            }
            void set_angular_vel(float angular_vel_input)
            {
                angular_vel = angular_vel_input;
            }
        };

        struct WarthogState
        {
            float linear_vel;
            float angular_vel;
            Kinematics::State kinematics_estimated;
            WarthogState()
            {
            }

            WarthogState(float linear_vel_input, float angular_vel_input, const Kinematics::State& kinematics_estimated_val)
                : linear_vel(linear_vel_input), angular_vel(angular_vel_input), kinematics_estimated(kinematics_estimated_val)
            {
            }

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
        WarthogApiBase(const AirSimSettings::VehicleSetting* vehicle_settings, std::shared_ptr<SensorFactory> sensor_factory, const Kinematics::State& state, const Environment& environment)
        {
            initialize(vehicle_settings, sensor_factory, state, environment);
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

        virtual void setWarthogControls(const WarthogControls& controls) = 0;
        virtual void updateWarthogState(const WarthogState& state) = 0;
        virtual const WarthogState& getWarthogState() const = 0;
        virtual const WarthogControls& getWarthogControls() const = 0;

        virtual ~WarthogApiBase() = default;

        std::shared_ptr<const SensorFactory> sensor_factory_;
        SensorCollection sensors_;
        vector<shared_ptr<SensorBase>> sensor_storage_;

    protected:
        virtual void resetImplementation() override
        {
            getSensors().reset();
        }
    };
}
}
#endif // !air_WarthogApiBase_hpp
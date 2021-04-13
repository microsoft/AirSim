
#ifndef msr_AirLibUnitTests_SensorTest_hpp
#define msr_AirLibUnitTests_SensorTest_hpp

#include <chrono>

#include "sensors/SensorFactory.hpp"
#include "common/SteppableClock.hpp"
#include "common/common_utils/Stats.hpp"
#include "TestBase.hpp"

namespace msr {
    namespace airlib {

        class SensorTest : public TestBase
        {
        public:
            virtual void run() override
            {
                auto clock = std::make_shared<SteppableClock>(3E-3f);
                ClockFactory::get(clock);

                AirSimSettings::SensorSetting settings;
                settings.sensor_type = SensorBase::SensorType::Barometer;
                settings.enabled = true;
                settings.sensor_name = "baro1";                
                SensorFactory sensor_factory;
                auto barometer = sensor_factory.createSensorFromSettings(&settings);

                testAssert(barometer != nullptr, "barometer is null");

                // setup GroundTruth.
                std::unique_ptr<msr::airlib::Kinematics> kinematics;
                std::unique_ptr<msr::airlib::Environment> environment;
                Kinematics::State initial_kinematic_state = Kinematics::State::zero();;
                initial_kinematic_state.pose = Pose();
                kinematics.reset(new Kinematics(initial_kinematic_state));

                Environment::State initial_environment;
                initial_environment.position = initial_kinematic_state.pose.position;
                initial_environment.geo_point = GeoPoint();
                environment.reset(new Environment(initial_environment));

                barometer->initialize(&(kinematics->getState()), environment.get());
                barometer->reset();
                
                Utils::getSetMinLogLevel(true, 100);

                BarometerBase* baro = dynamic_cast<BarometerBase*>(barometer.get());
                testAssert(baro != nullptr, "barometer is not of type BarometerBase");

                common_utils::Stats stats;

                const int max_steps = 1000000;
                for (size_t i = 0; i < max_steps; i++)
                {
                    barometer->update();
                    auto pressure = baro->getOutput().pressure;
                    if (i > 0) {
                        stats.insert(pressure);
                    }
                    clock->stepBy(1000); // one microsecond.
                }

                std::cout << "Barometric pressure sensor stats:" << std::endl;
                std::cout << "  mean   = " << stats.mean() << std::endl;
                std::cout << "  min    = " << stats.min() << std::endl;
                std::cout << "  max    = " << stats.max() << std::endl;
                std::cout << "  stddev = " << stats.standardDeviation() << std::endl;
            }

        private:
            std::vector<std::string> messages_;

        };


    }
}
#endif // msr_AirLibUnitTests_SensorTest_hpp
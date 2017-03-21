#pragma once

#include "sensors/imu/ImuSimple.hpp"
#include "common/Common.hpp"
#include <thread>
#include <ostream>

namespace msr { namespace airlib {


class StandALoneSensors {
public:
    static void createStaticData(std::ostream& output, float period, float total_duration)
    {
        auto kinematics = Kinematics::State::zero();
        msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint(), 0);
        msr::airlib::Environment environment(initial_environment);

        ImuSimple imu;
        imu.initialize(&kinematics, &environment);


        float interations = total_duration / period;

        double last = Utils::getTimeSinceEpoch();
        for (auto i = 0; i < interations; ++i) {
            const auto& imu_output = imu.getOutput();
            output << Utils::getTimeSinceEpoch() << "\t";
            output << imu_output.angular_velocity.x() << "\t" << imu_output.angular_velocity.y() << "\t" << imu_output.angular_velocity.z() << "\t";
            output << imu_output.linear_acceleration.x() << "\t" << imu_output.linear_acceleration.y() << "\t" << imu_output.linear_acceleration.z() << "\n";

            std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpoch() - last))); 

            float dt = static_cast<float>(Utils::getTimeSinceEpoch() - last);
            last = Utils::getTimeSinceEpoch();
            environment.update(dt);
            imu.update(dt);
        }
    }
};


}}
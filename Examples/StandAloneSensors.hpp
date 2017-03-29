#pragma once

#include "sensors/imu/ImuSimple.hpp"
#include "sensors/Barometer/BarometerSimple.hpp"
#include "sensors/Magnetometer/MagnetometerSimple.hpp"
#include "common/Common.hpp"
#include <thread>
#include <ostream>

namespace msr { namespace airlib {


class StandALoneSensors {
public:
    static void generateImuStaticData(std::ostream& output_stream, float period, float total_duration)
    {
        auto kinematics = Kinematics::State::zero();
        msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint(), 0);
        msr::airlib::Environment environment(initial_environment);

        ImuSimple imu;
        imu.initialize(&kinematics, &environment);


        float interations = total_duration / period;

        output_stream << std::fixed;

        double last = Utils::getTimeSinceEpoch();
        for (auto i = 0; i < interations; ++i) {
            const auto& output = imu.getOutput();
            output_stream << Utils::getTimeSinceEpoch() << "\t";
            output_stream << output.angular_velocity.x() << "\t" << output.angular_velocity.y() << "\t" << output.angular_velocity.z() << "\t";
            output_stream << output.linear_acceleration.x() << "\t" << output.linear_acceleration.y() << "\t" << output.linear_acceleration.z() << "\n";

            std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpoch() - last))); 

            float dt = static_cast<float>(Utils::getTimeSinceEpoch() - last);
            last = Utils::getTimeSinceEpoch();
            environment.update(dt);
            imu.update(dt);
        }
    }

    static void generateBarometerStaticData(std::ostream& output_stream, float period, float total_duration)
    {
        auto kinematics = Kinematics::State::zero();
        msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint(), 0);
        msr::airlib::Environment environment(initial_environment);

        BarometerSimple baro;
        baro.initialize(&kinematics, &environment);


        float interations = total_duration / period;

        output_stream << std::fixed;

        double last = Utils::getTimeSinceEpoch();
        for (auto i = 0; i < interations; ++i) {
            const auto& output = baro.getOutput();
            output_stream << Utils::getTimeSinceEpoch() << "\t";
            output_stream << output.pressure << "\t" << output.altitude << std::endl;


            std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpoch() - last))); 

            float dt = static_cast<float>(Utils::getTimeSinceEpoch() - last);
            last = Utils::getTimeSinceEpoch();
            environment.update(dt);
            baro.update(dt);
        }
    }


    static void generateBarometerDynamicData(std::ostream& output_stream, float period, float total_duration)
    {
        auto kinematics = Kinematics::State::zero();
        msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint(), 0);
        msr::airlib::Environment environment(initial_environment);

        BarometerSimple baro;
        baro.initialize(&kinematics, &environment);


        float interations_20s = 20.0f / period;

        output_stream << std::fixed;

        double last = Utils::getTimeSinceEpoch();
        bool which_alt = false;
        for(auto j = 0; j < 10; ++j) {
            for (auto i = 0; i < interations_20s; ++i) {
                const auto& output = baro.getOutput();
                output_stream << Utils::getTimeSinceEpoch() << "\t";
                output_stream << output.pressure << "\t" << output.altitude << "\t" << 
                    environment.getState().geo_point.altitude << std::endl;


                std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpoch() - last))); 

                float dt = static_cast<float>(Utils::getTimeSinceEpoch() - last);
                last = Utils::getTimeSinceEpoch();
                environment.update(dt);
                baro.update(dt);
            }

            which_alt = !which_alt;
            environment.setPosition(Vector3r(0, 0, which_alt ? -1.78f : 0));
        }
    }


    static void generateMagnetometerDataLoc(std::ostream& output_stream, float period, float total_duration)
    {
        output_stream << std::fixed;
        float interations = total_duration / period;
        double last = Utils::getTimeSinceEpoch();
        for (float pitch = 0; pitch < 2.1*M_PIf; pitch += M_PIf/2) {
        for (float roll = 0; roll < 2.1*M_PIf; roll += M_PIf/2) {
        for (float yaw = 0; yaw < 2.1*M_PIf; yaw += M_PIf/2) {

            auto kinematics = Kinematics::State::zero();
            kinematics.pose.orientation = VectorMath::toQuaternion(pitch, roll, yaw);
            //msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint(47.7631699f, -122.0685655f, 111.208f), 0);
            msr::airlib::Environment::State initial_environment(kinematics.pose.position, GeoPoint(47.6628040f, -122.1167039f, 7.564f), 0);
            msr::airlib::Environment environment(initial_environment);
            MagnetometerSimple mag;
            mag.initialize(&kinematics, &environment);

            for (auto i = 0; i < interations; ++i) {
                const auto& output = mag.getOutput();
                const auto& geo = environment.getState().geo_point;

                output_stream << Utils::getTimeSinceEpoch() << "\t";
                output_stream << output.magnetic_field_body.x() << "\t" << output.magnetic_field_body.y() <<  "\t" << output.magnetic_field_body.z();
                output_stream << "\t" << geo.latitude << "\t" << geo.longitude << "\t" << geo.altitude;
                output_stream << "\t" << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x()<< "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z();
                output_stream << std::endl;

                std::this_thread::sleep_for(std::chrono::duration<double>(period - (Utils::getTimeSinceEpoch() - last))); 
                float dt = static_cast<float>(Utils::getTimeSinceEpoch() - last);
                last = Utils::getTimeSinceEpoch();
                environment.update(dt);
                mag.update(dt);
            }
        }}}
    }
};


}}
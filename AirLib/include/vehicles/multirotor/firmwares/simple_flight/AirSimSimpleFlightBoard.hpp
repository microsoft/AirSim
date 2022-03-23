// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightBoard_hpp
#define msr_airlib_AirSimSimpleFlightBoard_hpp

#include <exception>
#include <vector>
#include "firmware/interfaces/IBoard.hpp"
#include "firmware/Params.hpp"
#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include "physics/Kinematics.hpp"

namespace msr
{
namespace airlib
{

    class AirSimSimpleFlightBoard : public simple_flight::IBoard
    {
    public:
        AirSimSimpleFlightBoard(const simple_flight::Params* params, const MultiRotorParams* vehicle_params)
            : params_(params), vehicle_params_(vehicle_params)
        {
            const std::string& imu_name = "";
            const std::string& barometer_name = "";
            const std::string& magnetometer_name = "";
            const std::string& gps_name = "";
            setSensors(imu_name,
                       barometer_name,
                       magnetometer_name,
                       gps_name);
        }

        //interface for simulator --------------------------------------------------------------------------------
        //for now we don't do any state estimation and use ground truth (i.e. assume perfect sensors)
        void setGroundTruthKinematics(const Kinematics::State* kinematics)
        {
            kinematics_ = kinematics;
        }

        //called to get o/p motor signal as float value
        real_T getMotorControlSignal(uint index) const
        {
            //convert PWM to scaled 0 to 1 control signal
            return static_cast<float>(motor_output_[index]);
        }

        //set current RC stick status
        void setInputChannel(uint index, real_T val)
        {
            input_channels_[index] = static_cast<float>(val);
        }

        void setIsRcConnected(bool is_connected)
        {
            is_connected_ = is_connected;
        }

    public:
        //Board interface implementation --------------------------------------------------------------------------

        virtual uint64_t micros() const override
        {
            return clock()->nowNanos() / 1000;
        }

        virtual uint64_t millis() const override
        {
            return clock()->nowNanos() / 1000000;
        }

        virtual float readChannel(uint16_t index) const override
        {
            return input_channels_[index];
        }

        virtual float getAvgMotorOutput() const override
        {
            return ((getMotorControlSignal(0) + getMotorControlSignal(1) + getMotorControlSignal(2) + getMotorControlSignal(3)) / 4);
        }

        virtual bool isRcConnected() const override
        {
            return is_connected_;
        }

        virtual void writeOutput(uint16_t index, float value) override
        {
            motor_output_[index] = value;
        }

        virtual void setLed(uint8_t index, int32_t color) override
        {
            //TODO: implement this
            unused(index);
            unused(color);
        }

        virtual void readAccel(float accel[3]) const override
        {
            const auto& linear_accel = VectorMath::transformToBodyFrame(kinematics_->accelerations.linear, kinematics_->pose.orientation);
            accel[0] = linear_accel.x();
            accel[1] = linear_accel.y();
            accel[2] = linear_accel.z();
        }

        virtual void readGyro(float gyro[3]) const override
        {
            const auto angular_vel = kinematics_->twist.angular; //angular velocity is already in body frame
            gyro[0] = angular_vel.x();
            gyro[1] = angular_vel.y();
            gyro[2] = angular_vel.z();
        }

        virtual void reset() override
        {
            IBoard::reset();

            motor_output_.assign(params_->motor.motor_count, 0);
            input_channels_.assign(params_->rc.channel_count, 0);
            is_connected_ = false;
        }

        virtual void update() override
        {
            IBoard::update();

            //no op for now
        }

        virtual bool checkImuIfNew() const override
        {
            return imu_->checkIfNew();
        }

        virtual bool checkBarometerIfNew() const override
        {
            return barometer_->checkIfNew();
        }

        virtual bool checkMagnetometerIfNew() const override
        {
            return magnetometer_->checkIfNew();
        }

        virtual bool checkGpsIfNew() const override
        {
            return gps_->checkIfNew();
        }

        virtual void readImuData(real_T accel[3], real_T gyro[3]) const override
        {
            accel[0] = imu_->getOutput().linear_acceleration.x();
            accel[1] = imu_->getOutput().linear_acceleration.y();
            accel[2] = imu_->getOutput().linear_acceleration.z();

            gyro[0] = imu_->getOutput().angular_velocity.x();
            gyro[1] = imu_->getOutput().angular_velocity.y();
            gyro[2] = imu_->getOutput().angular_velocity.z();
        }

        virtual void readBarometerData(real_T* altitude) const override
        {
            *altitude = barometer_->getOutput().altitude;
        }

        virtual void readMagnetometerData(real_T mag[3]) const override
        {
            mag[0] = magnetometer_->getOutput().magnetic_field_body.x();
            mag[1] = magnetometer_->getOutput().magnetic_field_body.y();
            mag[2] = magnetometer_->getOutput().magnetic_field_body.z();
        }

        virtual void readGpsData(double geo[3], real_T vel[3]) const override
        {
            geo[0] = gps_->getOutput().gnss.geo_point.longitude;
            geo[1] = gps_->getOutput().gnss.geo_point.latitude;
            geo[2] = gps_->getOutput().gnss.geo_point.altitude;

            vel[0] = gps_->getOutput().gnss.velocity.x();
            vel[1] = gps_->getOutput().gnss.velocity.y();
            vel[2] = gps_->getOutput().gnss.velocity.z();
        }

    private:
        const SensorCollection& getSensors()
        {
            return vehicle_params_->getSensors();
        }

        void setSensors(const std::string& imu_name,
                        const std::string& barometer_name,
                        const std::string& magnetometer_name,
                        const std::string& gps_name)
        {
            imu_ = static_cast<const ImuBase*>(findSensorByName(imu_name, SensorBase::SensorType::Imu));
            if (imu_ == nullptr) {
                //throw VehicleControllerException(Utils::stringf("No IMU with name %s exist on vehicle", imu_name.c_str()));
            }
            barometer_ = static_cast<const BarometerBase*>(findSensorByName(barometer_name, SensorBase::SensorType::Barometer));
            if (barometer_ == nullptr) {
                //throw VehicleControllerException(Utils::stringf("No barometer with name %s exist on vehicle", barometer_name.c_str()));
            }
            magnetometer_ = static_cast<const MagnetometerBase*>(findSensorByName(magnetometer_name, SensorBase::SensorType::Magnetometer));
            if (magnetometer_ == nullptr) {
                //throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));
            }
            gps_ = static_cast<const GpsBase*>(findSensorByName(gps_name, SensorBase::SensorType::Gps));
            if (gps_ == nullptr) {
                //throw VehicleControllerException(Utils::stringf("No gps with name %s exist on vehicle", gps_name.c_str()));
            }
        }

        const SensorBase* findSensorByName(const std::string& sensor_name, const SensorBase::SensorType type)
        {
            const SensorBase* sensor = nullptr;

            // Find sensor with the given name (for empty input name, return the first one found)
            // Not efficient but should suffice given small number of sensors
            uint count_sensors = getSensors().size(type);
            for (uint i = 0; i < count_sensors; i++) {
                const SensorBase* current_sensor = getSensors().getByType(type, i);
                if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == "")) {
                    sensor = current_sensor;
                    break;
                }
            }

            return sensor;
        }

        void sleep(double msec)
        {
            clock()->sleep_for(msec * 1000.0);
        }

        const ClockBase* clock() const
        {
            return ClockFactory::get();
        }

        ClockBase* clock()
        {
            return ClockFactory::get();
        }

    private:
        //motor outputs
        std::vector<float> motor_output_;
        std::vector<float> input_channels_;
        bool is_connected_;

        const simple_flight::Params* params_;
        const Kinematics::State* kinematics_;

        const MultiRotorParams* vehicle_params_;
        const ImuBase* imu_ = nullptr;
        const BarometerBase* barometer_ = nullptr;
        const MagnetometerBase* magnetometer_ = nullptr;
        const GpsBase* gps_ = nullptr;
    };
}
} //namespace
#endif

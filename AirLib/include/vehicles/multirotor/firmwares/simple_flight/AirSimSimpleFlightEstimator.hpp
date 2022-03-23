// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AirSimSimpleFlightEstimator_hpp
#define msr_airlib_AirSimSimpleFlightEstimator_hpp

#include "firmware/interfaces/CommonStructs.hpp"
#include "AirSimSimpleFlightCommon.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

    class AirSimSimpleFlightEstimator : public simple_flight::IStateEstimator
    {
    public:
        AirSimSimpleFlightEstimator(simple_flight::IEkf* ekf)
            : ekf_(ekf), ekf_enabled_(ekf_->checkEkfEnabled())
        {
        }

        virtual ~AirSimSimpleFlightEstimator() {}

        //for now we don't do any state estimation and use ground truth (i.e. assume perfect sensors)
        void setGroundTruthKinematics(const Kinematics::State* kinematics, const Environment* environment)
        {
            kinematics_ = kinematics;
            environment_ = environment;
        }

        virtual bool checkEkfEnabled() const override
        {
            return ekf_enabled_;
        }

        virtual simple_flight::Axis3r getAngles() const override
        {
            if (ekf_enabled_) {
                return getEkfAngles();
            }
            else {
                return getTrueAngles();
            }
        }

        virtual simple_flight::Axis3r getAngularVelocity() const override
        {
            if (ekf_enabled_) {
                auto ekf_measurements = getEkfMeasurements();
                return AirSimSimpleFlightCommon::toAxis3r(VectorMath::transformToWorldFrame(AirSimSimpleFlightCommon::toVector3r(ekf_measurements.gyro),
                                                                                            AirSimSimpleFlightCommon::toQuaternion(getOrientation()),
                                                                                            true));
            }
            else {
                return getTrueAngularVelocity();
            }
        }

        virtual simple_flight::Axis3r getPosition() const override
        {
            if (ekf_enabled_) {
                return getEkfPosition();
            }
            else {
                return getTruePosition();
            }
        }

        virtual simple_flight::Axis3r transformToBodyFrame(const simple_flight::Axis3r& world_frame_val) const override
        {
            if (ekf_enabled_) {
                const Vector3r& vec = AirSimSimpleFlightCommon::toVector3r(world_frame_val);
                const Vector3r& trans = VectorMath::transformToBodyFrame(vec, AirSimSimpleFlightCommon::toQuaternion(getOrientation()));
                return AirSimSimpleFlightCommon::toAxis3r(trans);
            }
            else {
                const Vector3r& vec = AirSimSimpleFlightCommon::toVector3r(world_frame_val);
                const Vector3r& trans = VectorMath::transformToBodyFrame(vec, kinematics_->pose.orientation);
                return AirSimSimpleFlightCommon::toAxis3r(trans);
            }
        }

        virtual simple_flight::Axis3r getLinearVelocity() const override
        {
            if (ekf_enabled_) {
                return getEkfLinearVelocity();
            }
            else {
                return getTrueLinearVelocity();
            }
        }

        virtual simple_flight::Axis4r getOrientation() const override
        {
            if (ekf_enabled_) {
                return getEkfOrientation();
            }
            else {
                return getTrueOrientation();
            }
        }

        virtual simple_flight::GeoPoint getGeoPoint() const override // TODO return this from measurement
        {
            return AirSimSimpleFlightCommon::toSimpleFlightGeoPoint(environment_->getState().geo_point);
        }

        virtual simple_flight::GeoPoint getHomeGeoPoint() const override
        {
            return AirSimSimpleFlightCommon::toSimpleFlightGeoPoint(environment_->getHomeGeoPoint());
        }

        virtual simple_flight::KinematicsState getKinematicsEstimated() const override
        {
            simple_flight::KinematicsState state;
            state.position = getPosition();
            state.orientation = getOrientation();
            state.linear_velocity = getLinearVelocity();
            state.angular_velocity = getAngularVelocity();
            state.linear_acceleration = AirSimSimpleFlightCommon::toAxis3r(kinematics_->accelerations.linear); // remove this
            state.angular_acceleration = AirSimSimpleFlightCommon::toAxis3r(kinematics_->accelerations.angular); // remove this

            return state;
        }

        // additional methods to get ground truth

        virtual simple_flight::SensorMeasurements getTrueMeasurements() const override
        {
            simple_flight::SensorMeasurements true_measurements;

            VectorMath::Vector3f linear_acceleration = kinematics_->accelerations.linear - environment_->getState().gravity;

            //acceleration is in world frame so transform to body frame
            linear_acceleration = VectorMath::transformToBodyFrame(linear_acceleration,
                                                                   kinematics_->pose.orientation,
                                                                   true);

            true_measurements.accel.x() = linear_acceleration.x();
            true_measurements.accel.y() = linear_acceleration.y();
            true_measurements.accel.z() = linear_acceleration.z();
            true_measurements.gyro.x() = kinematics_->twist.angular.x();
            true_measurements.gyro.y() = kinematics_->twist.angular.y();
            true_measurements.gyro.z() = kinematics_->twist.angular.z();
            true_measurements.gps_position.x() = kinematics_->pose.position.x();
            true_measurements.gps_position.y() = kinematics_->pose.position.y();
            true_measurements.gps_position.z() = kinematics_->pose.position.z();
            true_measurements.gps_velocity.x() = kinematics_->twist.linear.x();
            true_measurements.gps_velocity.y() = kinematics_->twist.linear.y();
            true_measurements.gps_velocity.z() = kinematics_->twist.linear.z();
            true_measurements.baro_altitude = -1.0f * kinematics_->pose.position.z();

            return true_measurements;
        }
        virtual simple_flight::Axis3r getTrueAngles() const override
        {
            simple_flight::Axis3r angles;
            VectorMath::toEulerianAngle(kinematics_->pose.orientation,
                                        angles.pitch(),
                                        angles.roll(),
                                        angles.yaw());
            //Utils::log(Utils::stringf("Ang Est:\t(%f, %f, %f)", angles.pitch(), angles.roll(), angles.yaw()));

            return angles;
        }
        virtual simple_flight::Axis3r getTrueAngularVelocity() const override
        {
            const auto& anguler = kinematics_->twist.angular;

            simple_flight::Axis3r conv;
            conv.x() = anguler.x();
            conv.y() = anguler.y();
            conv.z() = anguler.z();

            return conv;
        }
        virtual simple_flight::Axis3r getTruePosition() const override
        {
            return AirSimSimpleFlightCommon::toAxis3r(kinematics_->pose.position);
        }
        virtual simple_flight::Axis3r getTrueLinearVelocity() const override
        {
            return AirSimSimpleFlightCommon::toAxis3r(kinematics_->twist.linear);
        }
        virtual simple_flight::Axis4r getTrueOrientation() const override
        {
            return AirSimSimpleFlightCommon::toAxis4r(kinematics_->pose.orientation);
        }
        virtual simple_flight::KinematicsState getTrueKinematicsEstimated() const override
        {
            simple_flight::KinematicsState state;
            state.position = getTruePosition();
            state.orientation = getTrueOrientation();
            state.linear_velocity = getTrueLinearVelocity();
            state.angular_velocity = getTrueAngularVelocity();
            state.linear_acceleration = AirSimSimpleFlightCommon::toAxis3r(kinematics_->accelerations.linear); // remove this
            state.angular_acceleration = AirSimSimpleFlightCommon::toAxis3r(kinematics_->accelerations.angular); // remove this

            return state;
        }

        // additional methods to get Ekf results

        virtual simple_flight::SensorMeasurements getEkfMeasurements() const override
        {
            simple_flight::SensorMeasurements ekf_measurements;
            auto ekf_measurements_vector = ekf_->getEkfMeasurements();

            ekf_measurements.accel.x() = ekf_measurements_vector(0);
            ekf_measurements.accel.y() = ekf_measurements_vector(1);
            ekf_measurements.accel.z() = ekf_measurements_vector(2);
            ekf_measurements.gyro.x() = ekf_measurements_vector(3);
            ekf_measurements.gyro.y() = ekf_measurements_vector(4);
            ekf_measurements.gyro.z() = ekf_measurements_vector(5);
            ekf_measurements.gps_position.x() = ekf_measurements_vector(6);
            ekf_measurements.gps_position.y() = ekf_measurements_vector(7);
            ekf_measurements.gps_position.z() = ekf_measurements_vector(8);
            ekf_measurements.gps_velocity.x() = ekf_measurements_vector(9);
            ekf_measurements.gps_velocity.y() = ekf_measurements_vector(10);
            ekf_measurements.gps_velocity.z() = ekf_measurements_vector(11);
            ekf_measurements.baro_altitude = ekf_measurements_vector(12);
            ekf_measurements.magnetic_flux.x() = ekf_measurements_vector(13);
            ekf_measurements.magnetic_flux.y() = ekf_measurements_vector(14);
            ekf_measurements.magnetic_flux.z() = ekf_measurements_vector(15);

            return ekf_measurements;
        }

        virtual simple_flight::Axis3r getEkfPosition() const override
        {
            simple_flight::Axis3r position;
            auto ekf_states = ekf_->getEkfStates();
            position.x() = ekf_states(0);
            position.y() = ekf_states(1);
            position.z() = ekf_states(2);

            return position;
        }

        virtual simple_flight::Axis3r getEkfLinearVelocity() const override
        {
            simple_flight::Axis3r velocity;
            auto ekf_states = ekf_->getEkfStates();
            velocity.x() = ekf_states(3);
            velocity.y() = ekf_states(4);
            velocity.z() = ekf_states(5);

            return velocity;
        }

        virtual simple_flight::Axis4r getEkfOrientation() const override
        {
            simple_flight::Axis4r orientation;
            auto ekf_states = ekf_->getEkfStates();

            orientation.val4() = ekf_states(6);
            orientation.x() = ekf_states(7);
            orientation.y() = ekf_states(8);
            orientation.z() = ekf_states(9);

            return orientation;
        }

        virtual simple_flight::Axis3r getEkfAngles() const override
        {
            simple_flight::Axis3r angles;
            Quaternionr orientation;

            auto ekf_states = ekf_->getEkfStates();
            orientation.w() = ekf_states(6);
            orientation.x() = ekf_states(7);
            orientation.y() = ekf_states(8);
            orientation.z() = ekf_states(9);

            VectorMath::toEulerianAngle(orientation,
                                        angles.pitch(),
                                        angles.roll(),
                                        angles.yaw());

            return angles;
        }

        virtual simple_flight::SensorBiases getEkfSensorBias() const override
        {
            simple_flight::SensorBiases bias;
            auto ekf_states = ekf_->getEkfStates();
            bias.accel.x() = ekf_states(10);
            bias.accel.y() = ekf_states(11);
            bias.accel.z() = ekf_states(12);
            bias.gyro.x() = ekf_states(13);
            bias.gyro.y() = ekf_states(14);
            bias.gyro.z() = ekf_states(15);
            bias.barometer = ekf_states(16);

            return bias;
        }

        virtual simple_flight::EkfKinematicsState getEkfKinematicsEstimated() const override
        {
            simple_flight::EkfKinematicsState state;
            state.position = getEkfPosition();
            state.orientation = getEkfOrientation();
            state.angles = getEkfAngles();
            state.linear_velocity = getEkfLinearVelocity();
            state.sensor_bias = getEkfSensorBias();

            return state;
        }

        virtual simple_flight::Axis3r getEkfPositionVariance() const override
        {
            simple_flight::Axis3r position_var;
            auto ekf_covariance = ekf_->getEkfCovariance();
            position_var.x() = ekf_covariance(0, 0);
            position_var.y() = ekf_covariance(1, 1);
            position_var.z() = ekf_covariance(2, 2);

            return position_var;
        }

        virtual simple_flight::Axis3r getEkfLinearVelocityVariance() const override
        {
            simple_flight::Axis3r velocity_var;
            auto ekf_covariance = ekf_->getEkfCovariance();
            velocity_var.x() = ekf_covariance(3, 3);
            velocity_var.y() = ekf_covariance(4, 4);
            velocity_var.z() = ekf_covariance(5, 5);

            return velocity_var;
        }

        virtual simple_flight::Axis4r getEkfOrientationVariance() const override
        {
            simple_flight::Axis4r orientation_var;
            auto ekf_covariance = ekf_->getEkfCovariance();
            orientation_var.val4() = ekf_covariance(6, 6);
            orientation_var.x() = ekf_covariance(7, 7);
            orientation_var.y() = ekf_covariance(8, 8);
            orientation_var.z() = ekf_covariance(9, 9);

            return orientation_var;
        }

        virtual simple_flight::Axis3r getEkfAnglesVariance() const override
        {
            simple_flight::Axis3r angles_var;
            auto ekf_angles_covariance = ekf_->getEkfEulerAnglesCovariance();
            angles_var.x() = ekf_angles_covariance(0, 0);
            angles_var.y() = ekf_angles_covariance(1, 1);
            angles_var.z() = ekf_angles_covariance(2, 2);

            return angles_var;
        }

        virtual simple_flight::Axis3r getEkfAccelBiasVariance() const override
        {
            simple_flight::Axis3r accel_bias_var;
            auto ekf_covariance = ekf_->getEkfCovariance();
            accel_bias_var.x() = ekf_covariance(10, 10);
            accel_bias_var.y() = ekf_covariance(11, 11);
            accel_bias_var.z() = ekf_covariance(12, 12);

            return accel_bias_var;
        }

        virtual simple_flight::Axis3r getEkfGyroBiasVariance() const override
        {
            simple_flight::Axis3r gyro_bias_var;
            auto ekf_covariance = ekf_->getEkfCovariance();
            gyro_bias_var.x() = ekf_covariance(13, 13);
            gyro_bias_var.y() = ekf_covariance(14, 14);
            gyro_bias_var.z() = ekf_covariance(15, 15);

            return gyro_bias_var;
        }

        virtual float getEkfBaroBiasVariance() const override
        {
            float gyro_bias_var;
            auto ekf_covariance = ekf_->getEkfCovariance();
            gyro_bias_var = ekf_covariance(16, 16);
            // baro_bias_cov = ekf_covariance(15, 15);

            return gyro_bias_var;
        }

        virtual simple_flight::EkfKinematicsState getEkfStateVariance() const override
        {
            simple_flight::EkfKinematicsState state_var;
            state_var.position = getEkfPositionVariance();
            state_var.orientation = getEkfOrientationVariance();
            state_var.angles = getEkfAnglesVariance();
            state_var.linear_velocity = getEkfLinearVelocityVariance();
            state_var.sensor_bias.accel = getEkfAccelBiasVariance();
            state_var.sensor_bias.gyro = getEkfGyroBiasVariance();
            state_var.sensor_bias.barometer = getEkfBaroBiasVariance();

            return state_var;
        }

        virtual float getEkfOrientationNorm() const override
        {
            float norm;
            Quaternionr orientation;

            auto ekf_states = ekf_->getEkfStates();
            norm = ekf_states(6) * ekf_states(6) + ekf_states(7) * ekf_states(7) + ekf_states(8) * ekf_states(8) + ekf_states(9) * ekf_states(9);
            norm = sqrt(norm);

            return norm;
        }

        virtual std::array<float, 6> getEkfOrientationOffDiagCovariance() const override
        {
            std::array<float, 6> ori_offdiag_cov;
            auto ekf_covariance = ekf_->getEkfCovariance();
            ori_offdiag_cov.at(0) = ekf_covariance(6, 7);
            ori_offdiag_cov.at(1) = ekf_covariance(6, 8);
            ori_offdiag_cov.at(2) = ekf_covariance(6, 9);
            ori_offdiag_cov.at(3) = ekf_covariance(7, 8);
            ori_offdiag_cov.at(4) = ekf_covariance(7, 9);
            ori_offdiag_cov.at(5) = ekf_covariance(8, 9);

            return ori_offdiag_cov;
        }

        virtual std::array<float, 12> getEkfOrientationGyroBiasCovariance() const override
        {
            std::array<float, 12> ori_gyro_bias_cov;
            auto ekf_covariance = ekf_->getEkfCovariance();
            ori_gyro_bias_cov.at(0) = ekf_covariance(6, 13);
            ori_gyro_bias_cov.at(1) = ekf_covariance(6, 14);
            ori_gyro_bias_cov.at(2) = ekf_covariance(6, 15);
            ori_gyro_bias_cov.at(3) = ekf_covariance(7, 13);
            ori_gyro_bias_cov.at(4) = ekf_covariance(7, 14);
            ori_gyro_bias_cov.at(5) = ekf_covariance(7, 15);
            ori_gyro_bias_cov.at(6) = ekf_covariance(8, 13);
            ori_gyro_bias_cov.at(7) = ekf_covariance(8, 14);
            ori_gyro_bias_cov.at(8) = ekf_covariance(8, 15);
            ori_gyro_bias_cov.at(9) = ekf_covariance(9, 13);
            ori_gyro_bias_cov.at(10) = ekf_covariance(9, 14);
            ori_gyro_bias_cov.at(11) = ekf_covariance(9, 15);

            return ori_gyro_bias_cov;
        }

    private:
        const Kinematics::State* kinematics_;
        const Environment* environment_;
        const simple_flight::IEkf* ekf_;
        const bool ekf_enabled_;
    };
}
} //namespace
#endif

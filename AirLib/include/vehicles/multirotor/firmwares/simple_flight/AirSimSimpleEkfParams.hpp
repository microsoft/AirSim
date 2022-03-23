
#ifndef msr_airlib_AirSimSimpleEkfParams_hpp
#define msr_airlib_AirSimSimpleEkfParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"
#include <cmath>

namespace msr
{
namespace airlib
{

    struct AirSimSimpleEkfParams
    {
        bool ekf_enabled = false;
        bool fuse_gps = false;
        bool fuse_baro = false;
        bool fuse_mag = false;

        struct Gyroscope
        {
            Vector3r std_error = Vector3r(0.2f, 0.2f, 0.2f);
        } gyro;

        struct Accelerometer
        {
            Vector3r std_error = Vector3r(0.1f, 0.1f, 0.1f);
        } accel;

        struct Gps
        {
            Vector3r std_error_position = Vector3r(5.0f, 5.0f, 10.0f);
            Vector3r std_error_velocity = Vector3r(2.0f, 2.0f, 5.0f);
        } gps;

        struct Barometer
        {
            real_T std_error = 1.0f;
        } baro;

        struct Magnetometer
        {
            Vector3r std_error = Vector3r(0.1f, 0.1f, 0.1f);
        } mag;

        struct PseudoMeasurement
        {
            real_T quaternion_norm_R = 0.00001f;
        } pseudo_meas;

        struct InitialStatesStdErr
        {
            Vector3r position = Vector3r(5.0f, 5.0f, 5.0f);
            Vector3r linear_velocity = Vector3r(2.0, 2.0, 5.0);
            // Vector3r attitude = Vector3r(2.0, 2.0, 5.0);
            Vector3r accel_bias = Vector3r(0.1f, 0.1f, 0.1f);
            Vector3r gyro_bias = Vector3r(0.005f, 0.005f, 0.005f);
            real_T baro_bias = 0.1f;
            Quaternionr quaternion = Quaternionr(0.03f, 0.03f, 0.03f, 0.03f);
        } initial_states_std_err;

        struct InitialStatesErr
        {
            Vector3r position = Vector3r(0.0f, 0.0f, 0.0f);
            Vector3r linear_velocity = Vector3r(0.0f, 0.0f, 0.0f);
            Vector3r attitude = Vector3r(0.0, 0.0, 0.0);
            Vector3r accel_bias = Vector3r(0.0f, 0.0f, 0.0f);
            Vector3r gyro_bias = Vector3r(0.0f, 0.0f, 0.0f);
            real_T baro_bias = 0.0f;
            Quaternionr quaternion;
        } initial_states_err;

        void readVector3r(const Settings& json_child, const std::array<std::string, 3>& json_str, Vector3r& vector)
        {
            float element_x = json_child.getFloat(json_str.at(0), Utils::nan<float>());
            if (!std::isnan(element_x)) {
                vector.x() = element_x;
            }
            float element_y = json_child.getFloat(json_str.at(1), Utils::nan<float>());
            if (!std::isnan(element_y)) {
                vector.y() = element_y;
            }
            float element_z = json_child.getFloat(json_str.at(2), Utils::nan<float>());
            if (!std::isnan(element_z)) {
                vector.z() = element_z;
            }
        }

        void readRealT(const Settings& json_child, const std::string json_str, real_T& destination)
        {
            float element = json_child.getFloat(json_str, Utils::nan<float>());
            if (!std::isnan(element)) {
                destination = element;
            }
        }

        void readQuaternionr(const Settings& json_child, const std::array<std::string, 4>& json_str, Quaternionr& quaternion)
        {
            readRealT(json_child, json_str.at(0), quaternion.w());
            float element_x = json_child.getFloat(json_str.at(1), Utils::nan<float>());
            if (!std::isnan(element_x)) {
                quaternion.x() = element_x;
            }
            float element_y = json_child.getFloat(json_str.at(2), Utils::nan<float>());
            if (!std::isnan(element_y)) {
                quaternion.y() = element_y;
            }
            float element_z = json_child.getFloat(json_str.at(3), Utils::nan<float>());
            if (!std::isnan(element_z)) {
                quaternion.z() = element_z;
            }
        }

        void initializeParameters(const AirSimSettings::EkfSetting* settings)
        {
            initializeFromSettings(settings);
            refreshAndUnitConversion();
        }

        void refreshAndUnitConversion()
        {
            gyro.std_error = gyro.std_error * M_PI / 180; // deg/s to rad/s
            initial_states_err.gyro_bias = initial_states_err.gyro_bias * M_PI / 180; // deg/s to rad/s
            initial_states_std_err.gyro_bias = initial_states_std_err.gyro_bias * M_PI / 180; // deg/s to rad/s

            initial_states_err.attitude = initial_states_err.attitude * M_PI / 180; //deg to rad
            initial_states_err.quaternion = VectorMath::toQuaternion(initial_states_err.attitude.y(),
                                                                     initial_states_err.attitude.x(),
                                                                     initial_states_err.attitude.z());
        }

        void initializeFromSettings(const AirSimSettings::EkfSetting* settings)
        {
            if (settings == nullptr) {
                return;
            }
            const auto& json = settings->settings;

            float enabled = json.getBool("Enabled", Utils::nan<bool>());
            if (!std::isnan(enabled)) {
                ekf_enabled = enabled;
            }
            float gps_fusion = json.getBool("GpsFusion", Utils::nan<bool>());
            if (!std::isnan(gps_fusion)) {
                fuse_gps = gps_fusion;
            }
            float baro_fusion = json.getBool("BaroFusion", Utils::nan<bool>());
            if (!std::isnan(baro_fusion)) {
                fuse_baro = baro_fusion;
            }
            float mag_fusion = json.getBool("MagnetoFusion", Utils::nan<bool>());
            if (!std::isnan(mag_fusion)) {
                fuse_mag = mag_fusion;
            }
            Settings imu_child;
            if (json.getChild("Imu", imu_child)) {
                std::array<std::string, 3> gyro_str = {
                    "GyroErrorStdDevX",
                    "GyroErrorStdDevY",
                    "GyroErrorStdDevZ"
                };
                readVector3r(imu_child, gyro_str, gyro.std_error);
                std::array<std::string, 3> accel_str = {
                    "AccelErrorStdDevX",
                    "AccelErrorStdDevY",
                    "AccelErrorStdDevZ"
                };
                readVector3r(imu_child, accel_str, accel.std_error);
            }
            Settings gps_child;
            if (json.getChild("Gps", gps_child)) {
                std::array<std::string, 3> gps_pos_str = {
                    "PositionErrorStdDevX",
                    "PositionErrorStdDevY",
                    "PositionErrorStdDevZ"
                };
                readVector3r(gps_child, gps_pos_str, gps.std_error_position);
                std::array<std::string, 3> gps_vel_str = {
                    "VelocityErrorStdDevX",
                    "VelocityErrorStdDevY",
                    "VelocityErrorStdDevZ"
                };
                readVector3r(gps_child, gps_vel_str, gps.std_error_velocity);
            }
            Settings baro_child;
            if (json.getChild("Barometer", baro_child)) {
                readRealT(baro_child, "PositionErrorStdDevZ", baro.std_error);
            }
            Settings mag_child;
            if (json.getChild("Magnetometer", mag_child)) {
                std::array<std::string, 3> mag_str = {
                    "MagFluxErrorStdDevX",
                    "MagFluxErrorStdDevY",
                    "MagFluxErrorStdDevZ"
                };
                readVector3r(mag_child, mag_str, mag.std_error);
            }
            Settings pseudo_meas_child;
            if (json.getChild("PseudoMeasurement", pseudo_meas_child)) {
                readRealT(pseudo_meas_child, "QuaternionNormR", pseudo_meas.quaternion_norm_R);
            }
            Settings initial_states_std_err_child;
            if (json.getChild("InitialStatesStdErr", initial_states_std_err_child)) {
                std::array<std::string, 3> pos_str = {
                    "PositionX",
                    "PositionY",
                    "PositionZ"
                };
                readVector3r(initial_states_std_err_child, pos_str, initial_states_std_err.position);
                std::array<std::string, 3> lin_vel_str = {
                    "LinearVelocityX",
                    "LinearVelocityY",
                    "LinearVelocityZ"
                };
                readVector3r(initial_states_std_err_child, lin_vel_str, initial_states_std_err.linear_velocity);
                std::array<std::string, 4> quaternion_str = {
                    "QuaternionW",
                    "QuaternionX",
                    "QuaternionY",
                    "QuaternionZ"
                };
                readQuaternionr(initial_states_std_err_child, quaternion_str, initial_states_std_err.quaternion);
                std::array<std::string, 3> accel_bias_str = {
                    "AccelBiasX",
                    "AccelBiasY",
                    "AccelBiasZ"
                };
                readVector3r(initial_states_std_err_child, accel_bias_str, initial_states_std_err.accel_bias);
                std::array<std::string, 3> gyro_bias_str = {
                    "GyroBiasX",
                    "GyroBiasY",
                    "GyroBiasZ"
                };
                readVector3r(initial_states_std_err_child, gyro_bias_str, initial_states_std_err.gyro_bias);
                readRealT(initial_states_std_err_child, "BaroBias", initial_states_std_err.baro_bias);
            }
            Settings initial_states_err_child;
            if (json.getChild("InitialStatesErr", initial_states_err_child)) {
                std::array<std::string, 3> pos_str = {
                    "PositionX",
                    "PositionY",
                    "PositionZ"
                };
                readVector3r(initial_states_err_child, pos_str, initial_states_err.position);
                std::array<std::string, 3> lin_vel_str = {
                    "LinearVelocityX",
                    "LinearVelocityY",
                    "LinearVelocityZ"
                };
                readVector3r(initial_states_err_child, lin_vel_str, initial_states_err.linear_velocity);
                std::array<std::string, 3> attitude_str = {
                    "AttitideRoll",
                    "AttitidePitch",
                    "AttitideYaw"
                };
                readVector3r(initial_states_err_child, attitude_str, initial_states_err.attitude);
                std::array<std::string, 3> accel_bias_str = {
                    "AccelBiasX",
                    "AccelBiasY",
                    "AccelBiasZ"
                };
                readVector3r(initial_states_err_child, accel_bias_str, initial_states_err.accel_bias);
                std::array<std::string, 3> gyro_bias_str = {
                    "GyroBiasX",
                    "GyroBiasY",
                    "GyroBiasZ"
                };
                readVector3r(initial_states_err_child, gyro_bias_str, initial_states_err.gyro_bias);
                readRealT(initial_states_err_child, "BaroBias", initial_states_err.baro_bias);
            }
        }
    };

}
} //namespace
#endif

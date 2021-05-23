// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ArduRoverCarController_hpp
#define msr_airlib_ArduRoverCarController_hpp

#include "vehicles/car/api/CarApiBase.hpp"
#include "sensors/SensorCollection.hpp"

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/distance/DistanceSimple.hpp"
#include "sensors/lidar/LidarSimple.hpp"

#include "UdpSocket.hpp"

#include <sstream>
#include <iostream>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace msr
{
namespace airlib
{

    class ArduRoverApi : public CarApiBase
    {

    public:
        ArduRoverApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
                     const Kinematics::State& state, const Environment& environment, const msr::airlib::GeoPoint& home_geopoint)
            : CarApiBase(vehicle_setting, sensor_factory, state, environment), home_geopoint_(home_geopoint)
        {
            connection_info_ = static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting)->connection_info;
            sensors_ = &getSensors();

            connect();
        }

        ~ArduRoverApi()
        {
            closeConnections();
        }

    protected:
        virtual void resetImplementation() override
        {
            CarApiBase::resetImplementation();
        }

    public:
        // Update sensor data & send to Ardupilot
        virtual void update() override
        {
            CarApiBase::update();

            sendSensors();
            recvRoverControl();
        }

        virtual const SensorCollection& getSensors() const override
        {
            return CarApiBase::getSensors();
        }

        // TODO: VehicleApiBase implementation
        virtual bool isApiControlEnabled() const override
        {
            // Return true so that CarPawnSim gets control message from external firmware and not keyboard
            return true;
        }

        virtual void enableApiControl(bool is_enabled) override
        {
            Utils::log("enableApiControl() - Not Implemented", Utils::kLogLevelInfo);
            unused(is_enabled);
        }

        virtual bool armDisarm(bool arm) override
        {
            Utils::log("armDisarm() - Not Implemented", Utils::kLogLevelInfo);
            unused(arm);
            return false;
        }

        virtual GeoPoint getHomeGeoPoint() const override
        {
            return home_geopoint_;
        }

        virtual void getStatusMessages(std::vector<std::string>& messages) override
        {
            unused(messages);
        }

    public:
        virtual void setCarControls(const CarControls& controls) override
        {
            // Currently ArduPilot vehicles don't implement AirSim movement APIs
            unused(controls);
        }

        virtual void updateCarState(const CarState& car_state) override
        {
            last_car_state_ = car_state;
        }

        virtual const CarState& getCarState() const override
        {
            return last_car_state_;
        }

        virtual const CarControls& getCarControls() const override
        {
            return last_controls_;
        }

    protected:
        void closeConnections()
        {
            if (udp_socket_ != nullptr)
                udp_socket_->close();
        }

        void connect()
        {
            port_ = static_cast<uint16_t>(connection_info_.udp_port);
            ip_ = connection_info_.udp_address;

            closeConnections();

            if (ip_ == "") {
                throw std::invalid_argument("UdpIp setting is invalid.");
            }

            if (port_ == 0) {
                throw std::invalid_argument("UdpPort setting has an invalid value.");
            }

            Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", port_, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);
            Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.control_port_local, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);

            udp_socket_ = std::make_unique<mavlinkcom::UdpSocket>();
            udp_socket_->bind(connection_info_.local_host_ip, connection_info_.control_port_local);
        }

    private:
        void recvRoverControl()
        {
            // Receive motor data
            RoverControlMessage pkt;
            int recv_ret = udp_socket_->recv(&pkt, sizeof(pkt), 100);
            while (recv_ret != sizeof(pkt)) {
                if (recv_ret <= 0) {
                    Utils::log(Utils::stringf("Error while receiving rotor control data - ErrorNo: %d", recv_ret), Utils::kLogLevelInfo);
                }
                else {
                    Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes", recv_ret, sizeof(pkt)), Utils::kLogLevelInfo);
                }

                recv_ret = udp_socket_->recv(&pkt, sizeof(pkt), 100);
            }

            last_controls_.throttle = pkt.throttle;
            last_controls_.steering = pkt.steering;

            if (pkt.throttle > 0) {
                last_controls_.is_manual_gear = false;
                last_controls_.manual_gear = 0;
            }
            else {
                last_controls_.is_manual_gear = true;
                last_controls_.manual_gear = -1;
            }
        }

        void sendSensors()
        {
            if (sensors_ == nullptr)
                return;

            std::ostringstream oss;

            const uint count_gps_sensors = sensors_->size(SensorBase::SensorType::Gps);
            if (count_gps_sensors != 0) {
                const auto& gps_output = getGpsData("");

                oss << ","
                       "\"gps\": {"
                    << std::fixed << std::setprecision(7)
                    << "\"lat\": " << gps_output.gnss.geo_point.latitude << ","
                    << "\"lon\": " << gps_output.gnss.geo_point.longitude << ","
                    << std::setprecision(3) << "\"alt\": " << gps_output.gnss.geo_point.altitude
                    << "},"

                    << "\"velocity\": {"
                    << std::setprecision(12)
                    << "\"world_linear_velocity\": ["
                    << gps_output.gnss.velocity[0] << ","
                    << gps_output.gnss.velocity[1] << ","
                    << gps_output.gnss.velocity[2] << "]"
                                                      "}";
            }

            // Send Distance Sensors data if present
            const uint count_distance_sensors = sensors_->size(SensorBase::SensorType::Distance);
            if (count_distance_sensors != 0) {
                // Start JSON element
                oss << ","
                       "\"rng\": {"
                       "\"distances\": [";

                // Used to avoid trailing comma
                std::string sep = "";

                // Add sensor outputs in the array
                for (uint i = 0; i < count_distance_sensors; ++i) {
                    const auto* distance_sensor = static_cast<const DistanceSimple*>(
                        sensors_->getByType(SensorBase::SensorType::Distance, i));
                    // Don't send the data if sending to external controller is disabled in settings
                    if (distance_sensor && distance_sensor->getParams().external_controller) {
                        const auto& distance_output = distance_sensor->getOutput();
                        // AP uses meters so no need to convert here
                        oss << sep << distance_output.distance;
                        sep = ",";
                    }
                }

                // Close JSON array & element
                oss << "]}";
            }

            const uint count_lidars = sensors_->size(SensorBase::SensorType::Lidar);
            if (count_lidars != 0) {
                oss << ","
                       "\"lidar\": {"
                       "\"point_cloud\": [";

                // Add sensor outputs in the array
                for (uint i = 0; i < count_lidars; ++i) {
                    const auto* lidar = static_cast<const LidarSimple*>(sensors_->getByType(SensorBase::SensorType::Lidar, i));

                    if (lidar && lidar->getParams().external_controller) {
                        const auto& lidar_output = lidar->getOutput();
                        std::copy(lidar_output.point_cloud.begin(), lidar_output.point_cloud.end(), std::ostream_iterator<real_T>(oss, ","));
                        // AP backend only takes in a single Lidar sensor data currently
                        break;
                    }
                }

                oss << "]}";
            }

            const auto& imu_output = getImuData("");

            float yaw;
            float pitch;
            float roll;
            VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);

            // UDP packets have a maximum size limit of 65kB
            char buf[65000];

            int ret = snprintf(buf, sizeof(buf), "{"
                                                 "\"timestamp\": %" PRIu64 ","
                                                 "\"imu\": {"
                                                 "\"angular_velocity\": [%.12f, %.12f, %.12f],"
                                                 "\"linear_acceleration\": [%.12f, %.12f, %.12f]"
                                                 "},"
                                                 "\"pose\": {"
                                                 "\"roll\": %.12f,"
                                                 "\"pitch\": %.12f,"
                                                 "\"yaw\": %.12f"
                                                 "}"
                                                 "%s"
                                                 "}\n",
                               static_cast<uint64_t>(ClockFactory::get()->nowNanos() / 1.0E3),
                               imu_output.angular_velocity[0],
                               imu_output.angular_velocity[1],
                               imu_output.angular_velocity[2],
                               imu_output.linear_acceleration[0],
                               imu_output.linear_acceleration[1],
                               imu_output.linear_acceleration[2],
                               roll,
                               pitch,
                               yaw,
                               oss.str().c_str());

            if (ret == -1) {
                Utils::log("Error while allocating memory for sensor message", Utils::kLogLevelError);
                return;
            }
            else if (static_cast<uint>(ret) >= sizeof(buf)) {
                Utils::log(Utils::stringf("Sensor message truncated, lost %d bytes", ret - sizeof(buf)), Utils::kLogLevelWarn);
            }

            // Send data
            if (udp_socket_ != nullptr) {
                udp_socket_->sendto(buf, strlen(buf), ip_, port_);
            }
        }

    private:
        struct RoverControlMessage
        {
            float throttle;
            float steering;
        };

        AirSimSettings::MavLinkConnectionInfo connection_info_;

        std::unique_ptr<mavlinkcom::UdpSocket> udp_socket_;

        uint16_t port_;
        std::string ip_;

        const SensorCollection* sensors_;

        CarControls last_controls_;
        GeoPoint home_geopoint_;
        CarState last_car_state_;
    };
}
} // namespace

#endif

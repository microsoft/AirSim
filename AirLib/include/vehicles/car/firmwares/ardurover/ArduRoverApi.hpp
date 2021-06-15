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
            if (sensors_ == nullptr || udp_socket_ == nullptr)
                return;

            std::ostringstream buf;

            // Start of JSON element
            buf << "{";

            buf << "\"timestamp\": " << ClockFactory::get()->nowNanos() / 1000 << ",";

            const auto& imu_output = getImuData("");

            buf << "\"imu\": {"
                << std::fixed << std::setprecision(7)
                << "\"angular_velocity\": ["
                << imu_output.angular_velocity[0] << ","
                << imu_output.angular_velocity[1] << ","
                << imu_output.angular_velocity[2] << "]"
                << ","
                << "\"linear_acceleration\": ["
                << imu_output.linear_acceleration[0] << ","
                << imu_output.linear_acceleration[1] << ","
                << imu_output.linear_acceleration[2] << "]"
                << "}";

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);

            buf << ","
                << "\"pose\": {"
                << "\"pitch\": " << pitch << ","
                << "\"roll\": " << roll << ","
                << "\"yaw\": " << yaw
                << "}";

            const uint count_gps_sensors = sensors_->size(SensorBase::SensorType::Gps);
            if (count_gps_sensors != 0) {
                const auto& gps_output = getGpsData("");

                buf << ","
                       "\"gps\": {"
                    << std::fixed << std::setprecision(7)
                    << "\"lat\": " << gps_output.gnss.geo_point.latitude << ","
                    << "\"lon\": " << gps_output.gnss.geo_point.longitude << ","
                    << std::setprecision(3) << "\"alt\": " << gps_output.gnss.geo_point.altitude
                    << "},"

                    << "\"velocity\": {"
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
                buf << ","
                       "\"rng\": {"
                       "\"distances\": [";

                // More than mm level accuracy isn't needed or used
                buf << std::fixed << std::setprecision(3);

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
                        buf << sep << distance_output.distance;
                        sep = ",";
                    }
                }

                // Close JSON array & element
                buf << "]}";
            }

            const uint count_lidars = sensors_->size(SensorBase::SensorType::Lidar);
            if (count_lidars != 0) {
                buf << ","
                       "\"lidar\": {"
                       "\"point_cloud\": [";

                // More than mm level accuracy isn't needed or used
                buf << std::fixed << std::setprecision(3);

                // Add sensor outputs in the array
                for (uint i = 0; i < count_lidars; ++i) {
                    const auto* lidar = static_cast<const LidarSimple*>(sensors_->getByType(SensorBase::SensorType::Lidar, i));

                    if (lidar && lidar->getParams().external_controller) {
                        const auto& lidar_output = lidar->getOutput();
                        std::copy(lidar_output.point_cloud.begin(), lidar_output.point_cloud.end(), std::ostream_iterator<real_T>(buf, ","));
                        // AP backend only takes in a single Lidar sensor data currently
                        break;
                    }
                }

                // Close JSON array & element
                buf << "]}";
            }

            // End of JSON, AP Parser needs newline
            buf << "}\n";

            // str copy is made since if later on something like -
            //  const char* ptr = buf.str().c_str()
            // is written, ptr is invalid since buf.str() is a temporary copy
            // Currently there's no way to get pointer to underlying buffer
            const std::string sensor_data = buf.str();
            udp_socket_->sendto(sensor_data.c_str(), sensor_data.length(), ip_, port_);
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

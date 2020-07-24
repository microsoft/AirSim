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
#include "sensors/lidar/LidarBase.hpp"

#include "UdpSocket.hpp"

#include <sstream>
#include <iostream>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace msr { namespace airlib {

class ArduRoverApi : public CarApiBase {

public:
    ArduRoverApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
                 const Kinematics::State& state, const Environment& environment, const msr::airlib::GeoPoint& home_geopoint)
    : CarApiBase(vehicle_setting, sensor_factory, state, environment),
      home_geopoint_(home_geopoint)
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
        if (udpSocket_ != nullptr)
            udpSocket_->close();
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
        Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.control_port, connection_info_.local_host_ip.c_str(), ip_.c_str()), Utils::kLogLevelInfo);

        udpSocket_ = std::make_shared<mavlinkcom::UdpSocket>();
        udpSocket_->bind(connection_info_.local_host_ip, connection_info_.control_port);
    }

private:
    void recvRoverControl()
    {
        // Receive motor data
        RoverControlMessage pkt;
        int recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
        while (recv_ret != sizeof(pkt)) {
            if (recv_ret <= 0) {
                Utils::log(Utils::stringf("Error while receiving rotor control data - ErrorNo: %d", recv_ret), Utils::kLogLevelInfo);
            } else {
                Utils::log(Utils::stringf("Received %d bytes instead of %zu bytes", recv_ret, sizeof(pkt)), Utils::kLogLevelInfo);
            }

            recv_ret = udpSocket_->recv(&pkt, sizeof(pkt), 100);
        }

        last_controls_.throttle = pkt.throttle;
        last_controls_.steering = pkt.steering;

        if (pkt.throttle > 0) {
            last_controls_.is_manual_gear = false;
            last_controls_.manual_gear = 0;
        } else {
            last_controls_.is_manual_gear = true;
            last_controls_.manual_gear = -1;
        }
    }

    void sendSensors()
    {
        if (sensors_ == nullptr)
            return;

        const auto& gps_output = getGpsData("");
        const auto& imu_output = getImuData("");

        std::ostringstream oss;

        const uint count_lidars = getSensors().size(SensorBase::SensorType::Lidar);
        // TODO: Add bool value in settings to check whether to send lidar data or not
        // Since it's possible that we don't want to send the lidar data to Ardupilot but still have the lidar (maybe as a ROS topic)
        if (count_lidars != 0) {
            const auto& lidar_output = getLidarData("");
            oss << ","
                   "\"lidar\": {"
                   "\"point_cloud\": [";

            std::copy(lidar_output.point_cloud.begin(), lidar_output.point_cloud.end(), std::ostream_iterator<real_T>(oss, ","));
            oss << "]}";
        }

        float yaw;
        float pitch;
        float roll;
        VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);

        char buf[65000];

        // TODO: Split the following sensor packet formation into different parts for individual sensors

        // UDP packets have a maximum size limit of 65kB
        int ret = snprintf(buf, sizeof(buf),
                           "{"
                           "\"timestamp\": %" PRIu64 ","
                           "\"imu\": {"
                           "\"angular_velocity\": [%.12f, %.12f, %.12f],"
                           "\"linear_acceleration\": [%.12f, %.12f, %.12f]"
                           "},"
                           "\"gps\": {"
                           "\"lat\": %.7f,"
                           "\"lon\": %.7f,"
                           "\"alt\": %.3f"
                           "},"
                           "\"velocity\": {"
                           "\"world_linear_velocity\": [%.12f, %.12f, %.12f]"
                           "},"
                           "\"pose\": {"
                           "\"roll\": %.12f,"
                           "\"pitch\": %.12f,"
                           "\"yaw\": %.12f"
                           "}"
                           "%s"
                           "}\n",
                           static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E3),
                           imu_output.angular_velocity[0],
                           imu_output.angular_velocity[1],
                           imu_output.angular_velocity[2],
                           imu_output.linear_acceleration[0],
                           imu_output.linear_acceleration[1],
                           imu_output.linear_acceleration[2],
                           gps_output.gnss.geo_point.latitude,
                           gps_output.gnss.geo_point.longitude,
                           gps_output.gnss.geo_point.altitude,
                           gps_output.gnss.velocity[0],
                           gps_output.gnss.velocity[1],
                           gps_output.gnss.velocity[2],
                           roll, pitch, yaw,
                           oss.str().c_str());

        if (ret == -1) {
            Utils::log("Error while allocating memory for sensor message", Utils::kLogLevelInfo);
            return;
        }
        else if (static_cast<uint>(ret) >= sizeof(buf)) {
            Utils::log(Utils::stringf("Sensor message truncated, lost %d bytes", ret - sizeof(buf)), Utils::kLogLevelInfo);
        }

        // Send data
        if (udpSocket_ != nullptr) {
            udpSocket_->sendto(buf, strlen(buf), ip_, port_);
        }
    }

private:
    struct RoverControlMessage {
        float throttle;
        float steering;
    };

    AirSimSettings::MavLinkConnectionInfo connection_info_;

    std::shared_ptr<mavlinkcom::UdpSocket> udpSocket_;

    uint16_t port_;
    std::string ip_;

    const SensorCollection* sensors_;

    CarControls last_controls_;
    GeoPoint home_geopoint_;
    CarState last_car_state_;
};

}} // namespace

#endif

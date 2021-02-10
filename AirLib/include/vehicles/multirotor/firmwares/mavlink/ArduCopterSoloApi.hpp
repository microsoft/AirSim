#ifndef msr_airlib_ArduCopterSoloApi_h
#define msr_airlib_ArduCopterSoloApi_h

#include "AdHocConnection.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp"

namespace msr { namespace airlib {


class ArduCopterSoloApi : public MavLinkMultirotorApi
{
public:
    virtual ~ArduCopterSoloApi()
    {
        closeAllConnection();
    }

    virtual void update()
    {
        if (sensors_ == nullptr)
            return;

        // send GPS and other sensor updates
        const uint count_gps_sensors = getSensors().size(SensorBase::SensorType::Gps);
        if (count_gps_sensors != 0) {
            const auto& gps_output = getGpsData("");
            const auto& imu_output = getImuData("");

            SensorMessage packet;
            packet.timestamp = clock()->nowNanos() / 1000;
            packet.latitude = gps_output.gnss.geo_point.latitude;
            packet.longitude = gps_output.gnss.geo_point.longitude;
            packet.altitude = gps_output.gnss.geo_point.altitude;

            common_utils::Utils::log("Current LLA: " + gps_output.gnss.geo_point.to_string(), common_utils::Utils::kLogLevelInfo);

            packet.speedN = gps_output.gnss.velocity[0];
            packet.speedE = gps_output.gnss.velocity[1];
            packet.speedD = gps_output.gnss.velocity[2];

            packet.xAccel = imu_output.linear_acceleration[0];
            packet.yAccel = imu_output.linear_acceleration[1];
            packet.zAccel = imu_output.linear_acceleration[2];

            float yaw;
            float pitch;
            float roll;
            VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);
            packet.yawDeg = yaw * 180.0 / M_PI;
            packet.pitchDeg = pitch * 180.0 / M_PI;
            packet.rollDeg = roll * 180.0 / M_PI;

            Vector3r bodyRPY(roll, pitch, yaw);

            // In the Unreal world, yaw is rotation around Z, so this seems to be RPY, like PySim
            Vector3r bodyVelocityRPY(imu_output.angular_velocity[0], imu_output.angular_velocity[1], imu_output.angular_velocity[2]);
            Vector3r earthRPY = bodyAnglesToEarthAngles(bodyRPY, bodyVelocityRPY);

            packet.rollRate = earthRPY[0] * 180.0 / M_PI;
            packet.pitchRate = earthRPY[1] * 180.0 / M_PI;
            packet.yawRate = earthRPY[2] * 180.0 / M_PI;

            // Heading appears to be unused by AruPilot.  But use yaw for now
            packet.heading = yaw;

            packet.airspeed = std::sqrt(
                packet.speedN * packet.speedN
                + packet.speedE * packet.speedE
                + packet.speedD * packet.speedD);

            packet.magic = 0x4c56414f;

            if (udpSocket_ != nullptr)
            {
                std::vector<uint8_t> msg(sizeof(packet));
                memcpy(msg.data(), &packet, sizeof(packet));
                udpSocket_->sendMessage(msg);
            }
        }
    }

    virtual void close()
    {
        MavLinkMultirotorApi::close();

        if (udpSocket_ != nullptr) {
            udpSocket_->close();
            udpSocket_->unsubscribe(rotorSubscriptionId_);
            udpSocket_ = nullptr;
        }
    }

protected:
    virtual void connect()
    {
        if (!is_simulation_mode_) {

            MavLinkMultirotorApi::connect();
        }
        else {
            const std::string& ip = connection_info_.udp_address;
            int port = connection_info_.udp_port;

            close();

            if (ip == "") {
                throw std::invalid_argument("UdpIp setting is invalid.");
            }

            if (port == 0) {
                throw std::invalid_argument("UdpPort setting has an invalid value.");
            }

            Utils::log(Utils::stringf("Using UDP port %d, local IP %s, remote IP %s for sending sensor data", port, connection_info_.local_host_ip.c_str(), ip.c_str()), Utils::kLogLevelInfo);
            Utils::log(Utils::stringf("Using UDP port %d for receiving rotor power", connection_info_.control_port, connection_info_.local_host_ip.c_str(), ip.c_str()), Utils::kLogLevelInfo);

            udpSocket_ = mavlinkcom::AdHocConnection::connectLocalUdp("ArduCopterSoloConnector", ip, connection_info_.control_port);
            mavlinkcom::AdHocMessageHandler handler = [this](std::shared_ptr<mavlinkcom::AdHocConnection> connection, const std::vector<uint8_t> &msg) {
                this->rotorPowerMessageHandler(connection, msg);
            };

            rotorSubscriptionId_ = udpSocket_->subscribe(handler);
        }
}

private:
#ifdef __linux__
    struct __attribute__((__packed__)) SensorMessage {
#else
#pragma pack(push,1)
    struct SensorMessage {
#endif
        // this is the packet sent by the simulator
        // to the APM executable to update the simulator state
        // All values are little-endian
        uint64_t timestamp;
        double latitude, longitude; // degrees
        double altitude;  // MSL
        double heading;   // degrees
        double speedN, speedE, speedD; // m/s
        double xAccel, yAccel, zAccel;       // m/s/s in body frame
        double rollRate, pitchRate, yawRate; // degrees/s/s in earth frame
        double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
        double airspeed; // m/s
        uint32_t magic; // 0x4c56414f
    };
#ifndef __linux__
#pragma pack(pop)
#endif

    static const int kArduCopterRotorControlCount = 11;

    struct RotorControlMessage {
        // ArduPilot Solo rotor control datagram format
        uint16_t pwm[kArduCopterRotorControlCount];
        uint16_t speed, direction, turbulance;
    };

    std::shared_ptr<mavlinkcom::AdHocConnection> udpSocket_;
    int rotorSubscriptionId_;

    virtual void normalizeRotorControls()
    {
        // change 1000-2000 to 0-1.
        for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
            rotor_controls_[i] = (rotor_controls_[i] - 1000.0f) / 1000.0f;
        }
    }

    void rotorPowerMessageHandler(std::shared_ptr<mavlinkcom::AdHocConnection> connection, const std::vector<uint8_t> &msg)
    {
        if (msg.size() != sizeof(RotorControlMessage))
        {
            Utils::log("Got rotor control message of size " + std::to_string(msg.size()) + " when we were expecting size " + std::to_string(sizeof(RotorControlMessage)), Utils::kLogLevelError);
            return;
        }

        RotorControlMessage rotorControlMessage;
        memcpy(&rotorControlMessage, msg.data(), sizeof(RotorControlMessage));

        std::lock_guard<std::mutex> guard_actuator(hil_controls_mutex_);    //use same mutex as HIL_CONTROl

        for (auto i = 0; i < RotorControlsCount && i < kArduCopterRotorControlCount; ++i) {
            rotor_controls_[i] = rotorControlMessage.pwm[i];
        }

        normalizeRotorControls();
    }

    Vector3r bodyAnglesToEarthAngles(Vector3r bodyRPY, Vector3r bodyVelocityRPY)
    {
        float p = bodyVelocityRPY[0];
        float q = bodyVelocityRPY[1];
        float r = bodyVelocityRPY[2];

        // Roll, pitch, yaw
        float phi = bodyRPY[0];
        float theta = bodyRPY[1];

        float phiDot = p + tan(theta)*(q*sin(phi) + r * cos(phi));

        float thetaDot = q * cos(phi) - r * sin(phi);
        if (fabs(cos(theta)) < 1.0e-20)
        {
            theta += 1.0e-10f;
        }

        float psiDot = (q*sin(phi) + r * cos(phi)) / cos(theta);

        return Vector3r(phiDot, thetaDot, psiDot);
    }

};

}
} //namespace

#endif

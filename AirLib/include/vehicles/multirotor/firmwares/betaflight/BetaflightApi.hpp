#ifndef msr_airlib_BetaflightDroneController_hpp
#define msr_airlib_BetaflightDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

// Sensors
#include "sensors/imu/ImuBase.hpp"

#include "UdpSocket.hpp"


namespace msr
{ 
namespace airlib 
{

    class BetaflightApi: public MultirotorApiBase
    {

    public:
        BetaflightApi(const MultiRotorParams* vehicle_params,  const AirSimSettings::MavLinkConnectionInfo& connection_info)
            : connection_info_(connection_info), vehicle_params_(vehicle_params)
        {
            connect();
        }

        ~BetaflightApi()
        {
            closeConnections();
        }

    public:
        virtual void resetImplementation() override
        {
            MultirotorApiBase::resetImplementation();

            // Reset state
        }

        // Update sensor data & send to betaflight
        virtual void update() override 
        {
            MultirotorApiBase::update();

            sendState();
            recvControl();
        }

        // necessary overrides
        virtual bool isApiControlEnabled() const override
        {
            Utils::log("Not Implemented: isApiControlEnabled", Utils::kLogLevelInfo);
            return false;
        }
        virtual void enableApiControl(bool is_enabled) override
        {
            Utils::log("Not Implemented: enableApiControl", Utils::kLogLevelInfo);
            unused(is_enabled);
        }
        virtual bool armDisarm(bool arm) override
        {
            Utils::log("Not Implemented: armDisarm", Utils::kLogLevelInfo);
            unused(arm);
            return false;
        }
        virtual GeoPoint getHomeGeoPoint() const override
        {
            Utils::log("Not Implemented: getHomeGeoPoint", Utils::kLogLevelInfo);
            return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
        }
        virtual void getStatusMessages(std::vector<std::string>& messages) override
        {
            unused(messages);
        }

        virtual const SensorCollection& getSensors() const override
        {
            return vehicle_params_->getSensors();
        }

    public: //TODO:MultirotorApiBase implementation
        virtual real_T getActuation(unsigned int rotor_index) const override
        {
            return rotor_controls_[rotor_index];
        }

        virtual size_t getActuatorCount() const override
        {
            return vehicle_params_->getParams().rotor_count;
        }

        virtual void moveByRC(const RCData& rc_data) override
        {
            unused(rc_data);
            // setRCData(rc_data);
        }

        virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
        {
            Utils::log("Not Implemented: setSimulatedGroundTruth", Utils::kLogLevelInfo);
            unused(kinematics);
            unused(environment);
        }

        virtual bool setRCData(const RCData& rc_data) override
        {
            last_rcData_ = rc_data;
            is_rc_connected_ = true;

            return true;
        }

    protected:
        virtual Kinematics::State getKinematicsEstimated() const override
        {
            Utils::log("Not Implemented: getKinematicsEstimated", Utils::kLogLevelInfo);
            Kinematics::State state;
            return state;
        }

        virtual Vector3r getPosition() const override
        {
            Utils::log("Not Implemented: getPosition", Utils::kLogLevelInfo);
            return Vector3r(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
        }

        virtual Vector3r getVelocity() const override
        {
            Utils::log("Not Implemented: getVelocity", Utils::kLogLevelInfo);
            return Vector3r(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
        }

        virtual Quaternionr getOrientation() const override
        {
            Utils::log("Not Implemented: getOrientation", Utils::kLogLevelInfo);
            return Quaternionr(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
        }

        virtual LandedState getLandedState() const override
        {
            Utils::log("Not Implemented: getLandedState", Utils::kLogLevelInfo);
            return LandedState::Landed;
        }

        virtual RCData getRCData() const override
        {
            //return what we received last time through setRCData
            return last_rcData_;
        }

        virtual GeoPoint getGpsLocation() const override
        {
            Utils::log("Not Implemented: getGpsLocation", Utils::kLogLevelInfo);
            return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
        }

        virtual float getCommandPeriod() const override
        {
            return 1.0f / 50; //50hz
        }

        virtual float getTakeoffZ() const override
        {
            // pick a number, 3 meters is probably safe
            // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
            // return params_.takeoff.takeoff_z;
            return 3.0;
        }

        virtual float getDistanceAccuracy() const override
        {
            return 0.5f; //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
        }

        virtual void setControllerGains(uint8_t controllerType, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
        {
            unused(controllerType);
            unused(kp);
            unused(ki);
            unused(kd);
            Utils::log("Not Implemented: setControllerGains", Utils::kLogLevelInfo);
        }

        virtual void commandMotorPWMs(float front_right_pwm, float front_left_pwm, float rear_right_pwm, float rear_left_pwm) override
        {
            unused(front_right_pwm);
            unused(front_left_pwm);
            unused(rear_right_pwm);
            unused(rear_left_pwm);
            Utils::log("Not Implemented: commandMotorPWMs", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw_rate);
            unused(throttle);
            Utils::log("Not Implemented: commandRollPitchYawrateThrottle", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw);
            unused(z);
            Utils::log("Not Implemented: commandRollPitchYawZ", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw);
            unused(throttle);
            Utils::log("Not Implemented: commandRollPitchYawThrottle", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) override
        {
            unused(roll);
            unused(pitch);
            unused(yaw_rate);
            unused(z);
            Utils::log("Not Implemented: commandRollPitchYawrateZ", Utils::kLogLevelInfo);
        }

        virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) override
        {
            unused(roll_rate);
            unused(pitch_rate);
            unused(yaw_rate);
            unused(z);
            Utils::log("Not Implemented: commandAngleRatesZ", Utils::kLogLevelInfo);
        }

        virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) override
        {
            unused(roll_rate);
            unused(pitch_rate);
            unused(yaw_rate);
            unused(throttle);
            Utils::log("Not Implemented: commandAngleRatesZ", Utils::kLogLevelInfo);
        }

        virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
        {
            unused(vx);
            unused(vy);
            unused(vz);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandVelocity", Utils::kLogLevelInfo);
        }

        virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
        {
            unused(vx);
            unused(vy);
            unused(z);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandVelocityZ", Utils::kLogLevelInfo);
        }

        virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
        {
            unused(x);
            unused(y);
            unused(z);
            unused(yaw_mode);
            Utils::log("Not Implemented: commandPosition", Utils::kLogLevelInfo);
        }

        virtual const MultirotorApiParams& getMultirotorApiParams() const override
        {
            return safety_params_;
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
        // actual functions
        virtual void normalizeRotorControls()
        {
            // change 1000-2000 to 0-1.
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = (rotor_controls_[i] - 1000.0f) / 1000.0f;
            }
        }

        void sendState() // send fdmPacket to betaflight i.e udp:9002
        {   
            FdmPacket pkt;

            // Time
            pkt.timestamp = ClockFactory::get()->nowNanos() / 1E9;
            
            // IMU
            const auto& imu_output = getImuData("");
                // Angular Velocity
                pkt.imu_angular_velocity_rpy[0] = imu_output.angular_velocity[0];
                pkt.imu_angular_velocity_rpy[1] = imu_output.angular_velocity[1];
                pkt.imu_angular_velocity_rpy[2] = imu_output.angular_velocity[2];

                // Linear Acceleration
                pkt.imu_linear_acceleration_xyz[0] = imu_output.linear_acceleration[0];
                pkt.imu_linear_acceleration_xyz[1] = imu_output.linear_acceleration[1];
                pkt.imu_linear_acceleration_xyz[2] = imu_output.linear_acceleration[2];

                // Orientation Quaternion. In case USE_IMU_CALC is not defined on betaflight side
                pkt.imu_orientation_quat[0] = imu_output.orientation.w();
                pkt.imu_orientation_quat[1] = imu_output.orientation.x();
                pkt.imu_orientation_quat[2] = imu_output.orientation.y();
                pkt.imu_orientation_quat[3] = imu_output.orientation.z();
            // GPS

                // Position

                // Velocity
            if (udp_socket_ != nullptr) {
                udp_socket_->sendto(&pkt, sizeof(pkt), ip_, port_);
            }
        }

        void recvControl()
        {
            ServoPacket pkt;
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
            for (auto i = 0; i < kBetaflightRotorControlCount; ++i) {
                rotor_controls_[i] = pkt.pwm[i];
            }

            normalizeRotorControls();
           // Utils::log(Utils::stringf("pwm received: [ %f, %f, %f, %f ]", rotor_controls_[0], rotor_controls_[1], rotor_controls_[2], rotor_controls_[3]), Utils::kLogLevelInfo);
        }

    private:
        
        struct FdmPacket // equivalent of fdm_packet in betaflight SITL
        {  
            double timestamp;                   // in seconds
            double imu_angular_velocity_rpy[3]; // rad/s -> range: +/- 8192; +/- 2000 deg/se
            double imu_linear_acceleration_xyz[3];    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
            double imu_orientation_quat[4];     // w, x, y, z
            double velocity_xyz[3];             // m/s, earth frame
            double position_xyz[3];             // meters, NED from origin
        };
        
        static const int kBetaflightRotorControlCount = 4;
        
        struct ServoPacket // equivalent of servo_packet in betaflight SITL
        {  
            u_int16_t pwm[kBetaflightRotorControlCount]; 
        };

        float rotor_controls_[kBetaflightRotorControlCount];

        std::unique_ptr<mavlinkcom::UdpSocket> udp_socket_;
        MultirotorApiParams safety_params_;
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        const MultiRotorParams* vehicle_params_;

        uint16_t port_;
        std::string ip_;
        RCData last_rcData_;
        bool is_rc_connected_;
    };
}
} // namespace 

#endif
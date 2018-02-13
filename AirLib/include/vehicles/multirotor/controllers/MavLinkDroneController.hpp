// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MavLinkDroneController_hpp
#define msr_airlib_MavLinkDroneController_hpp

#include "MavLinkVehicle.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkNode.hpp"
#include "MavLinkVideoStream.hpp"

#include <queue>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <exception>

#include "common/Common.hpp"
#include "common/common_utils/Timer.hpp"
#include "common/CommonStructs.hpp"
#include "common/VectorMath.hpp"
#include "vehicles/multirotor//MultiRotor.hpp"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "controllers/PidController.hpp"

//sensors
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/distance/DistanceBase.hpp"

namespace msr { namespace airlib {


class MavLinkDroneController : public DroneControllerBase
{
public:
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::MultiRotor MultiRotor;

    struct ConnectionInfo {
        /* Default values are requires so uninitialized instance doesn't have random values */

        bool use_serial = true; // false means use UDP instead
                                //Used to connect via HITL: needed only if use_serial = true
        std::string serial_port = "*";
        int baud_rate = 115200;

        //Used to connect to drone over UDP: needed only if use_serial = false
        std::string ip_address = "127.0.0.1";
        int ip_port = 14560;

        // The PX4 SITL app requires receiving drone commands over a different mavlink channel.
        // So set this to empty string to disable this separate command channel.
        std::string sitl_ip_address = "127.0.0.1";
        int sitl_ip_port = 14556;

        // The log viewer can be on a different machine, so you can configure it's ip address and port here.
        int logviewer_ip_port = 14388;
        int logviewer_ip_sport = 14389; // for logging all messages we send to the vehicle.
        std::string logviewer_ip_address = "127.0.0.1";

        // The QGroundControl app can be on a different machine, so you can configure it's ip address and port here.
        int qgc_ip_port = 14550;
        std::string qgc_ip_address = "127.0.0.1";

        // mavlink vehicle identifiers
        uint8_t sim_sysid = 142;
        int sim_compid = 42;
        uint8_t offboard_sysid = 134;
        int offboard_compid = 1;
        uint8_t vehicle_sysid = 135;
        int vehicle_compid = 1;

        // if you want to select a specific local network adapter so you can reach certain remote machines (e.g. wifi versus ethernet) 
        // then you will want to change the LocalHostIp accordingly.  This default only works when log viewer and QGC are also on the
        // same machine.  Whatever network you choose it has to be the same one for external
        std::string local_host_ip = "127.0.0.1";

        std::string model = "Generic";
    };

public:
    //required for pimpl
    MavLinkDroneController();
    virtual ~MavLinkDroneController();

    //non-base interface specific to MavLinKDroneController
    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation);
    ConnectionInfo getMavConnectionInfo();
    static std::string findPX4();

    //TODO: get rid of below methods?
    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height);
    bool hasVideoRequest();

    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override;
    virtual void update() override;
    virtual size_t getVertexCount() override;
    virtual real_T getVertexControlSignal(unsigned int rotor_index) override;
    virtual void getStatusMessages(std::vector<std::string>& messages) override;
    virtual bool isAvailable(std::string& message) override;
    virtual bool isApiControlEnabled() override;
    virtual bool isSimulationMode() override;
    virtual void enableApiControl(bool is_enabled) override;
    virtual void setSimulationMode(bool is_set) override;
    virtual Pose getDebugPose() override;
    //*** End: VehicleControllerBase implementation ***//


    //*** Start: DroneControllerBase implementation ***//
public:
    virtual Kinematics::State getKinematicsEstimated() override;
    virtual Vector3r getPosition() override;
    virtual Vector3r getVelocity() override;
    virtual Quaternionr getOrientation() override;
    virtual LandedState getLandedState() override;
    virtual RCData getRCData() override;
    virtual void setRCData(const RCData& rcData) override;

    virtual bool armDisarm(bool arm, CancelableBase& cancelable_action) override;
    virtual bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override;
    virtual bool land(float max_wait_seconds, CancelableBase& cancelable_action) override;
    virtual bool goHome(CancelableBase& cancelable_action) override;
    virtual bool hover(CancelableBase& cancelable_action) override;
    virtual GeoPoint getHomeGeoPoint() override;
    virtual GeoPoint getGpsLocation() override;
    virtual void reportTelemetry(float renderTime) override;

    virtual float getCommandPeriod() override;
    virtual float getTakeoffZ() override;
    virtual float getDistanceAccuracy() override;

    virtual bool loopCommandPre() override;
    virtual void loopCommandPost() override;
protected:
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override;
    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override;
    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override;
    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override;
    const VehicleParams& getVehicleParams() override;
    //*** End: DroneControllerBase implementation ***//

private: //pimpl
    struct impl;
    std::unique_ptr<impl> pimpl_;
};


class MavLinkLogViewerLog : public mavlinkcom::MavLinkLog
{
public:
    MavLinkLogViewerLog(std::shared_ptr<mavlinkcom::MavLinkNode> proxy) {
        proxy_ = proxy;
    }
    void write(const mavlinkcom::MavLinkMessage& msg, uint64_t timestamp = 0) override {
        unused(timestamp);
        mavlinkcom::MavLinkMessage copy;
        ::memcpy(&copy, &msg, sizeof(mavlinkcom::MavLinkMessage));
        proxy_->sendMessage(copy);
    }

private:
    std::shared_ptr<mavlinkcom::MavLinkNode> proxy_;
};

struct MavLinkDroneController::impl {
public:
    static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
    static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
    static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
    static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
    static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board
    static const int RotorControlsCount = 8;
    static const int messageReceivedTimeout = 10; ///< Seconds 

    std::shared_ptr<mavlinkcom::MavLinkNode> logviewer_proxy_, logviewer_out_proxy_, qgc_proxy_;

    size_t status_messages_MaxSize = 5000;

    std::shared_ptr<mavlinkcom::MavLinkNode> hil_node_;
    std::shared_ptr<mavlinkcom::MavLinkConnection> connection_;
    std::shared_ptr<mavlinkcom::MavLinkVideoServer> video_server_;
    std::shared_ptr<DroneControllerBase> mav_vehicle_control_;

    mavlinkcom::MavLinkAttPosMocap MocapPoseMessage;
    mavlinkcom::MavLinkHeartbeat HeartbeatMessage;
    mavlinkcom::MavLinkSetMode SetModeMessage;
    mavlinkcom::MavLinkStatustext StatusTextMessage;
    mavlinkcom::MavLinkHilControls HilControlsMessage;
    mavlinkcom::MavLinkHilActuatorControls HilActuatorControlsMessage;
    mavlinkcom::MavLinkCommandLong CommandLongMessage;

    mavlinkcom::MavLinkHilSensor last_sensor_message_;
    mavlinkcom::MavLinkDistanceSensor last_distance_message_;
    mavlinkcom::MavLinkHilGps last_gps_message_;

    std::mutex mocap_pose_mutex_, heartbeat_mutex_, set_mode_mutex_, status_text_mutex_, hil_controls_mutex_, last_message_mutex_;
    MavLinkDroneController* parent_;

    impl(MavLinkDroneController* parent)
        : parent_(parent)
    {
    }

    //variables required for VehicleControllerBase implementation
    ConnectionInfo connection_info_;
    bool is_any_heartbeat_, is_hil_mode_set_, is_armed_;
    bool is_controls_0_1_; //Are motor controls specified in 0..1 or -1..1?
    float rotor_controls_[RotorControlsCount];
    std::queue<std::string> status_messages_;
    int hil_state_freq_;
    bool actuators_message_supported_;
    const SensorCollection* sensors_;    //this is optional
    uint64_t last_gps_time_;
    bool was_reset_;
    Pose debug_pose_;
    std::string is_available_message_;
    bool is_available_;

    //additional variables required for DroneControllerBase implementation
    //this is optional for methods that might not use vehicle commands
    std::shared_ptr<mavlinkcom::MavLinkVehicle> mav_vehicle_;
    int state_version_;
    mavlinkcom::VehicleState current_state;
    float target_height_;
    bool is_api_control_enabled_;
    bool is_simulation_mode_;
    PidController thrust_controller_;
    common_utils::Timer hil_message_timer_;
    common_utils::Timer sitl_message_timer_;

    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation)
    {
        connection_info_ = connection_info;
        sensors_ = sensors;
        is_simulation_mode_ = is_simulation;

        try {
            openAllConnections();
            is_available_ = true;
        }
        catch (std::exception& ex) {
            is_available_ = false;
            is_available_message_ = Utils::stringf("Failed to connect: %s", ex.what());
        }
    }

    bool isAvailable(std::string& message)
    {
        if (!is_available_)
            message = is_available_message_;
        return is_available_;
    }

    ConnectionInfo getMavConnectionInfo()
    {
        return connection_info_;
    }

    void normalizeRotorControls()
    {
        //if rotor controls are in not in 0-1 range then they are in -1 to 1 range in which case
        //we normalize them to 0 to 1 for PX4
        if (!is_controls_0_1_) {
            // change -1 to 1 to 0 to 1.
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = (rotor_controls_[i] + 1.0f) / 2.0f;
            }
        }
        else {
            //this applies to PX4 and may work differently on other firmwares. 
            //We use 0.2 as idle rotors which leaves out range of 0.8
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = Utils::clip(0.8f * rotor_controls_[i] + 0.20f, 0.0f, 1.0f);
            }
        }
    }

    void initializeMavSubscriptions()
    {
        if (connection_ != nullptr && mav_vehicle_ != nullptr) {
            is_any_heartbeat_ = false;
            is_hil_mode_set_ = false;
            is_armed_ = false;
            is_controls_0_1_ = true;
            Utils::setValue(rotor_controls_, 0.0f);
            //TODO: main_node_->setMessageInterval(...);
            connection_->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
                unused(connection);
                processMavMessages(msg);
            });

            // listen to the other mavlink connection also
            auto mavcon = mav_vehicle_->getConnection();
            if (mavcon != connection_) {
                mavcon->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
                    unused(connection);
                    processMavMessages(msg);
                });
            }
        }
    }

    bool sendTestMessage(std::shared_ptr<mavlinkcom::MavLinkNode> node) {
        try {
            // try and send a test message.
            mavlinkcom::MavLinkHeartbeat test;
            test.autopilot = static_cast<int>(mavlinkcom::MAV_AUTOPILOT::MAV_AUTOPILOT_PX4);
            test.type = static_cast<uint8_t>(mavlinkcom::MAV_TYPE::MAV_TYPE_GCS);
            test.base_mode = 0;
            test.custom_mode = 0;
            test.mavlink_version = 3;
            node->sendMessage(test);
            test.system_status = 0;
            return true;
        }
        catch (std::exception) {
            return false;
        }
    }

    bool connectToLogViewer()
    {
        //set up logviewer proxy
        if (connection_info_.logviewer_ip_address.size() > 0) {
            std::shared_ptr<mavlinkcom::MavLinkConnection> connection;
            createProxy("LogViewer", connection_info_.logviewer_ip_address, connection_info_.logviewer_ip_port, connection_info_.local_host_ip,
                logviewer_proxy_, connection);
            if (!sendTestMessage(logviewer_proxy_)) {
                // error talking to log viewer, so don't keep trying, and close the connection also.
                logviewer_proxy_->getConnection()->close();
                logviewer_proxy_ = nullptr;
            }

            std::shared_ptr<mavlinkcom::MavLinkConnection> out_connection;
            createProxy("LogViewerOut", connection_info_.logviewer_ip_address, connection_info_.logviewer_ip_sport, connection_info_.local_host_ip,
                logviewer_out_proxy_, out_connection);
            if (!sendTestMessage(logviewer_out_proxy_)) {
                // error talking to log viewer, so don't keep trying, and close the connection also.
                logviewer_out_proxy_->getConnection()->close();
                logviewer_out_proxy_ = nullptr;
            }
            else if (mav_vehicle_ != nullptr) {
                mav_vehicle_->getConnection()->startLoggingSendMessage(std::make_shared<MavLinkLogViewerLog>(logviewer_out_proxy_));
            }
        }
        return logviewer_proxy_ != nullptr;
    }

    bool connectToQGC()
    {
        if (connection_info_.qgc_ip_address.size() > 0) {
            std::shared_ptr<mavlinkcom::MavLinkConnection> connection;
            createProxy("QGC", connection_info_.qgc_ip_address, connection_info_.qgc_ip_port, connection_info_.local_host_ip, qgc_proxy_, connection);
            if (!sendTestMessage(qgc_proxy_)) {
                // error talking to QGC, so don't keep trying, and close the connection also.
                qgc_proxy_->getConnection()->close();
                qgc_proxy_ = nullptr;
            }
            else {
                connection->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection_val, const mavlinkcom::MavLinkMessage& msg) {
                    unused(connection_val);
                    processQgcMessages(msg);
                });
            }
        }
        return qgc_proxy_ != nullptr;
    }


    void createProxy(std::string name, std::string ip, int port, string local_host_ip,
        std::shared_ptr<mavlinkcom::MavLinkNode>& node, std::shared_ptr<mavlinkcom::MavLinkConnection>& connection)
    {
        if (connection_ == nullptr)
            throw std::domain_error("MavLinkDroneController requires connection object to be set before createProxy call");

        connection = mavlinkcom::MavLinkConnection::connectRemoteUdp("Proxy to: " + name + " at " + ip + ":" + std::to_string(port), local_host_ip, ip, port);

        // it is ok to reuse the simulator sysid and compid here because this node is only used to send a few messages directly to this endpoint
        // and all other messages are funnelled through from PX4 via the Join method below.
        node = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        node->connect(connection);

        // now join the main connection to this one, this causes all PX4 messages to be sent to the proxy and all messages from the proxy will be
        // send directly to the PX4 (using whatever sysid/compid comes from that remote node).
        connection_->join(connection);

        auto mavcon = mav_vehicle_->getConnection();
        if (mavcon != connection_) {
            mavcon->join(connection);
        }
    }

    static std::string findPX4()
    {
        auto result = mavlinkcom::MavLinkConnection::findSerialPorts(0, 0);
        for (auto iter = result.begin(); iter != result.end(); iter++)
        {
            mavlinkcom::SerialPortInfo info = *iter;
            if (
                (
                (info.vid == pixhawkVendorId) &&
                    (info.pid == pixhawkFMUV4ProductId || info.pid == pixhawkFMUV2ProductId || info.pid == pixhawkFMUV2OldBootloaderProductId)
                    ) ||
                    (
                (info.displayName.find(L"PX4_") != std::string::npos)
                        )
                )
            {
                // printf("Auto Selecting COM port: %S\n", info.displayName.c_str());
                return std::string(info.portName.begin(), info.portName.end());
            }
        }
        return "";
    }

    void connect()
    {
        createMavConnection(connection_info_);
        initializeMavSubscriptions();
    }

    void createMavConnection(const ConnectionInfo& connection_info)
    {
        if (connection_info.use_serial) {
            createMavSerialConnection(connection_info.serial_port, connection_info.baud_rate);
        }
        else {
            createMavUdpConnection(connection_info.ip_address, connection_info.ip_port);
        }
        //Uncomment below for sending images over MavLink
        //connectToVideoServer();
    }

    void createMavUdpConnection(const std::string& ip, int port)
    {
        close();

        if (ip == "") {
            throw std::invalid_argument("UdpIp setting is invalid.");
        }

        if (port == 0) {
            throw std::invalid_argument("UdpPort setting has an invalid value.");
        }

        addStatusMessage(Utils::stringf("Connecting to UDP port %d, local IP %s, remote IP...", port, connection_info_.local_host_ip.c_str(), ip.c_str()));
        connection_ = mavlinkcom::MavLinkConnection::connectRemoteUdp("hil", connection_info_.local_host_ip, ip, port);
        hil_node_ = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        hil_node_->connect(connection_);
        addStatusMessage(std::string("Connected over UDP."));

        mav_vehicle_ = std::make_shared<mavlinkcom::MavLinkVehicle>(connection_info_.vehicle_sysid, connection_info_.vehicle_compid);

        if (connection_info_.sitl_ip_address != "" && connection_info_.sitl_ip_port != 0 && connection_info_.sitl_ip_port != port) {
            // bugbug: the PX4 SITL mode app cannot receive commands to control the drone over the same mavlink connection
            // as the HIL_SENSOR messages, we must establish a separate mavlink channel for that so that DroneShell works.
            addStatusMessage(Utils::stringf("Connecting to PX4 SITL UDP port %d, local IP %s, remote IP...",
                connection_info_.sitl_ip_port, connection_info_.local_host_ip.c_str(), connection_info_.sitl_ip_address.c_str()));

            auto sitlconnection = mavlinkcom::MavLinkConnection::connectRemoteUdp("sitl",
                connection_info_.local_host_ip, connection_info_.sitl_ip_address, connection_info_.sitl_ip_port);
            mav_vehicle_->connect(sitlconnection);

            addStatusMessage(std::string("Connected to SITL over UDP."));
        }
        else {
            mav_vehicle_->connect(connection_);
        }

        mav_vehicle_->startHeartbeat();
    }

    void createMavSerialConnection(const std::string& port_name, int baud_rate)
    {
        close();

        std::string port_name_auto = port_name;
        if (port_name_auto == "" || port_name_auto == "*") {
            port_name_auto = findPX4();
            if (port_name_auto == "") {
                throw std::domain_error("Could not detect a connected PX4 flight controller on any USB ports. You can specify USB port in settings.json.");
            }
        }

        if (port_name_auto == "") {
            throw std::invalid_argument("USB port for PX4 flight controller is empty. Please set it in settings.json.");
        }

        if (baud_rate == 0) {
            throw std::invalid_argument("Baud rate specified in settings.json is 0 which is invalid");
        }

        addStatusMessage(Utils::stringf("Connecting to PX4 over serial port: %s, baud rate %d ....", port_name_auto.c_str(), baud_rate));
        connection_ = mavlinkcom::MavLinkConnection::connectSerial("hil", port_name_auto, baud_rate);
        connection_->ignoreMessage(mavlinkcom::MavLinkAttPosMocap::kMessageId); //TODO: find better way to communicate debug pose instead of using fake Mocap messages
        hil_node_ = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        hil_node_->connect(connection_);
        addStatusMessage("Connected to PX4 over serial port.");

        mav_vehicle_ = std::make_shared<mavlinkcom::MavLinkVehicle>(connection_info_.vehicle_sysid, connection_info_.vehicle_compid);
        mav_vehicle_->connect(connection_); // in this case we can use the same connection.
        mav_vehicle_->startHeartbeat();
    }

    mavlinkcom::MavLinkHilSensor getLastSensorMessage()
    {
        std::lock_guard<std::mutex> guard(last_message_mutex_);
        return last_sensor_message_;
    }

    mavlinkcom::MavLinkDistanceSensor getLastDistanceMessage()
    {
        std::lock_guard<std::mutex> guard(last_message_mutex_);
        return last_distance_message_;
    }

    mavlinkcom::MavLinkHilGps getLastGpsMessage()
    {
        std::lock_guard<std::mutex> guard(last_message_mutex_);
        return last_gps_message_;
    }

    void setArmed(bool armed)
    {
        is_armed_ = armed;
        if (!armed) {
            //reset motor controls
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = 0;
            }
        }
    }

    void processQgcMessages(const mavlinkcom::MavLinkMessage& msg)
    {
        if (msg.msgid == MocapPoseMessage.msgid) {
            std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
            MocapPoseMessage.decode(msg);
            getMocapPose(debug_pose_.position, debug_pose_.orientation);
        }
        //else ignore message
    }

    void addStatusMessage(const std::string& message)
    {
        std::lock_guard<std::mutex> guard_status(status_text_mutex_);
        //if queue became too large, clear it first
        if (status_messages_.size() > status_messages_MaxSize)
            Utils::clear(status_messages_, status_messages_MaxSize - status_messages_.size());
        status_messages_.push(message);
    }

    void processMavMessages(const mavlinkcom::MavLinkMessage& msg)
    {
        if (msg.msgid == HeartbeatMessage.msgid) {
            std::lock_guard<std::mutex> guard_heartbeat(heartbeat_mutex_);

            //TODO: have MavLinkNode track armed state so we don't have to re-decode message here again
            HeartbeatMessage.decode(msg);
            bool armed = (HeartbeatMessage.base_mode & static_cast<uint8_t>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED)) > 0;
            setArmed(armed);
            if (!is_any_heartbeat_) {
                is_any_heartbeat_ = true;
                if (HeartbeatMessage.autopilot == static_cast<uint8_t>(mavlinkcom::MAV_AUTOPILOT::MAV_AUTOPILOT_PX4) &&
                    HeartbeatMessage.type == static_cast<uint8_t>(mavlinkcom::MAV_TYPE::MAV_TYPE_FIXED_WING)) {
                    // PX4 will scale fixed wing servo outputs to -1 to 1
                    // and it scales multi rotor servo outpus to 0 to 1.
                    is_controls_0_1_ = false;
                }
            } else if (is_simulation_mode_ && !is_hil_mode_set_) {
                setHILMode();
            }
        } else if (msg.msgid == StatusTextMessage.msgid) {
            StatusTextMessage.decode(msg);
            //lock is established by below method
            addStatusMessage(std::string(StatusTextMessage.text));
        } else if (msg.msgid == CommandLongMessage.msgid) {
            CommandLongMessage.decode(msg);
            if (CommandLongMessage.command == static_cast<int>(mavlinkcom::MAV_CMD::MAV_CMD_SET_MESSAGE_INTERVAL)) {
                int msg_id = static_cast<int>(CommandLongMessage.param1 + 0.5);
                if (msg_id == 115) { //HIL_STATE_QUATERNION
                    hil_state_freq_ = static_cast<int>(CommandLongMessage.param2 + 0.5);
                }
            }
        } else if (msg.msgid == HilControlsMessage.msgid) {
            if (!actuators_message_supported_) {
                std::lock_guard<std::mutex> guard_controls(hil_controls_mutex_);

                HilControlsMessage.decode(msg);
                //is_arned_ = (HilControlsMessage.mode & 128) > 0; //TODO: is this needed?
                rotor_controls_[0] = HilControlsMessage.roll_ailerons;
                rotor_controls_[1] = HilControlsMessage.pitch_elevator;
                rotor_controls_[2] = HilControlsMessage.yaw_rudder;
                rotor_controls_[3] = HilControlsMessage.throttle;
                rotor_controls_[4] = HilControlsMessage.aux1;
                rotor_controls_[5] = HilControlsMessage.aux2;
                rotor_controls_[6] = HilControlsMessage.aux3;
                rotor_controls_[7] = HilControlsMessage.aux4;

                normalizeRotorControls();
            }
        }
        else if (msg.msgid == HilActuatorControlsMessage.msgid) {
            actuators_message_supported_ = true;

            std::lock_guard<std::mutex> guard_actuator(hil_controls_mutex_);    //use same mutex as HIL_CONTROl

            HilActuatorControlsMessage.decode(msg);
            //is_arned_ = (HilControlsMessage.mode & 128) > 0; //TODO: is this needed?
            for (auto i = 0; i < 8; ++i) {
                rotor_controls_[i] = HilActuatorControlsMessage.controls[i];
            }
            normalizeRotorControls();
        }
        //else ignore message
    }

    void sendHILSensor(const Vector3r& acceleration, const Vector3r& gyro, const Vector3r& mag, float abs_pressure, float pressure_alt)
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to send simulated sensor messages while not in simulation mode");

        mavlinkcom::MavLinkHilSensor hil_sensor;
        hil_sensor.time_usec = static_cast<uint64_t>(Utils::getTimeSinceEpochNanos() / 1000.0);
       
        hil_sensor.xacc = acceleration.x();
        hil_sensor.yacc = acceleration.y();
        hil_sensor.zacc = acceleration.z();
        hil_sensor.xgyro = gyro.x();
        hil_sensor.ygyro = gyro.y();
        hil_sensor.zgyro = gyro.z();

        hil_sensor.xmag = mag.x();
        hil_sensor.ymag = mag.y();
        hil_sensor.zmag = mag.z();

        hil_sensor.abs_pressure = abs_pressure;
        hil_sensor.pressure_alt = pressure_alt;
        //TODO: enable temperature? diff_pressure
        hil_sensor.fields_updated = was_reset_ ? (1 << 31) : 0;

        if (hil_node_ != nullptr) {
            hil_node_->sendMessage(hil_sensor);
        }

        std::lock_guard<std::mutex> guard(last_message_mutex_);
        last_sensor_message_ = hil_sensor;
    }

    void sendDistanceSensor(float min_distance, float max_distance, float current_distance, float sensor_type, float sensor_id, float orientation)
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to send simulated distance sensor messages while not in simulation mode");

        mavlinkcom::MavLinkDistanceSensor distance_sensor;
        distance_sensor.time_boot_ms = static_cast<uint32_t>(Utils::getTimeSinceEpochNanos() / 1000000.0);

        distance_sensor.min_distance = static_cast<uint16_t>(min_distance);
        distance_sensor.max_distance = static_cast<uint16_t>(max_distance);
        distance_sensor.current_distance = static_cast<uint16_t>(current_distance);
        distance_sensor.type = static_cast<uint8_t>(sensor_type);
        distance_sensor.id = static_cast<uint8_t>(sensor_id);
        distance_sensor.orientation = static_cast<uint8_t>(orientation);
        //TODO: use covariance parameter?

        if (hil_node_ != nullptr) {
            hil_node_->sendMessage(distance_sensor);
        }

        std::lock_guard<std::mutex> guard(last_message_mutex_);
        last_distance_message_ = distance_sensor;
    }

    void sendHILGps(const GeoPoint& geo_point, const Vector3r& velocity, float velocity_xy, float cog,
        float eph, float epv, int fix_type, unsigned int satellites_visible)
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to send simulated GPS messages while not in simulation mode");

        mavlinkcom::MavLinkHilGps hil_gps;
        hil_gps.time_usec = static_cast<uint64_t>(Utils::getTimeSinceEpochNanos() / 1000.0);
        hil_gps.lat = static_cast<int32_t>(geo_point.latitude * 1E7);
        hil_gps.lon = static_cast<int32_t>(geo_point.longitude* 1E7);
        hil_gps.alt = static_cast<int32_t>(geo_point.altitude * 1000);
        hil_gps.vn = static_cast<int16_t>(velocity.x() * 100);
        hil_gps.ve = static_cast<int16_t>(velocity.y() * 100);
        hil_gps.vd = static_cast<int16_t>(velocity.z() * 100);
        hil_gps.eph = static_cast<uint16_t>(eph * 100);
        hil_gps.epv = static_cast<uint16_t>(epv * 100);
        hil_gps.fix_type = static_cast<uint8_t>(fix_type);
        hil_gps.vel = static_cast<uint16_t>(velocity_xy * 100);
        hil_gps.cog = static_cast<uint16_t>(cog * 100);
        hil_gps.satellites_visible = static_cast<uint8_t>(satellites_visible);

        if (hil_node_ != nullptr) {
            hil_node_->sendMessage(hil_gps);
        }

        if (hil_gps.lat < 0.1f && hil_gps.lat > -0.1f) {
            //Utils::DebugBreak();
            Utils::log("hil_gps.lat was too close to 0", Utils::kLogLevelError);
        }

        std::lock_guard<std::mutex> guard(last_message_mutex_);
        last_gps_message_ = hil_gps;
    }

    real_T getVertexControlSignal(unsigned int rotor_index)
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to read simulated motor controls while not in simulation mode");

        std::lock_guard<std::mutex> guard(hil_controls_mutex_);
        return rotor_controls_[rotor_index];
    }

    void resetState()
    {
        //reset state
        is_any_heartbeat_ = is_hil_mode_set_ = is_armed_ = false;
        is_controls_0_1_ = true;
        hil_state_freq_ = -1;
        actuators_message_supported_ = false;
        last_gps_time_ = 0;
        state_version_ = 0;
        current_state = mavlinkcom::VehicleState();
        target_height_ = 0;
        is_api_control_enabled_ = false;
        thrust_controller_ = PidController();
        Utils::setValue(rotor_controls_, 0.0f);
        was_reset_ = false;
        debug_pose_ = Pose::nanPose();
    }

    //*** Start: VehicleControllerBase implementation ***//
    void reset()
    {
        resetState();
        was_reset_ = true;
        setNormalMode();
    }

    const ImuBase* getImu()
    {
        return static_cast<const ImuBase*>(sensors_->getByType(SensorBase::SensorType::Imu));
    }
    const MagnetometerBase* getMagnetometer()
    {
        return static_cast<const MagnetometerBase*>(sensors_->getByType(SensorBase::SensorType::Magnetometer));
    }
    const BarometerBase* getBarometer()
    {
        return static_cast<const BarometerBase*>(sensors_->getByType(SensorBase::SensorType::Barometer));
    }
    const DistanceBase* getDistance()
    {
        return static_cast<const DistanceBase*>(sensors_->getByType(SensorBase::SensorType::Distance));
    }
    const GpsBase* getGps()
    {
        return static_cast<const GpsBase*>(sensors_->getByType(SensorBase::SensorType::Gps));
    }

    void update()
    {
        if (sensors_ == nullptr || connection_ == nullptr || !connection_->isOpen())
            return;

        //send sensor updates
        const auto& imu_output = getImu()->getOutput();
        const auto& mag_output = getMagnetometer()->getOutput();
        const auto& baro_output = getBarometer()->getOutput();

        sendHILSensor(imu_output.linear_acceleration,
            imu_output.angular_velocity,
            mag_output.magnetic_field_body,
            baro_output.pressure * 0.01f /*Pa to Milibar */, baro_output.altitude);


        const auto * distance = getDistance();
        if (distance) {
          const auto& distance_output = distance->getOutput();
          float pitch, roll, yaw;
          VectorMath::toEulerianAngle(distance_output.relative_pose.orientation, pitch, roll, yaw);

          sendDistanceSensor(distance_output.min_distance / 100, //m -> cm
                             distance_output.max_distance / 100, //m -> cm
                             distance_output.distance,
                             0, //sensor type: //TODO: allow changing in settings?
                             77, //sensor id, //TODO: should this be something real?
                             pitch); //TODO: convert from radians to degrees?
        }

        const auto gps = getGps();
        if (gps != nullptr) {
            const auto& gps_output = gps->getOutput();

            //send GPS
            if (gps_output.is_valid && gps_output.gnss.time_utc > last_gps_time_) {
                last_gps_time_ = gps_output.gnss.time_utc;
                Vector3r gps_velocity = gps_output.gnss.velocity;
                Vector3r gps_velocity_xy = gps_velocity;
                gps_velocity_xy.z() = 0;
                float gps_cog;
                if (Utils::isApproximatelyZero(gps_velocity.y(), 1E-2f) && Utils::isApproximatelyZero(gps_velocity.x(), 1E-2f))
                    gps_cog = 0;
                else
                    gps_cog = Utils::radiansToDegrees(atan2(gps_velocity.y(), gps_velocity.x()));
                if (gps_cog < 0)
                    gps_cog += 360;

                sendHILGps(gps_output.gnss.geo_point, gps_velocity, gps_velocity_xy.norm(), gps_cog,
                    gps_output.gnss.eph, gps_output.gnss.epv, gps_output.gnss.fix_type, 10);
            }
        }

        //must be done at the end
        if (was_reset_)
            was_reset_ = false;
    }

    void getStatusMessages(std::vector<std::string>& messages)
    {
        messages.clear();
        std::lock_guard<std::mutex> guard(status_text_mutex_);

        while (!status_messages_.empty()) {
            messages.push_back(status_messages_.front());
            status_messages_.pop();
        }
    }

    void openAllConnections()
    {
        close(); //just in case if connections were open
        resetState(); //reset all variables we might have changed during last session

        connect();
        connectToLogViewer();
        connectToQGC();

    }
    void closeAllConnection()
    {
        close();
    }
    //*** End: VehicleControllerBase implementation ***//

    void getMocapPose(Vector3r& position, Quaternionr& orientation)
    {
        position.x() = MocapPoseMessage.x; position.y() = MocapPoseMessage.y; position.z() = MocapPoseMessage.z;
        orientation.w() = MocapPoseMessage.q[0]; orientation.x() = MocapPoseMessage.q[1];
        orientation.y() = MocapPoseMessage.q[2]; orientation.z() = MocapPoseMessage.q[3];
    }

    void sendCollision(float normalX, float normalY, float normalZ)
    {
        checkVehicle();

        mavlinkcom::MavLinkCollision collision{};
        collision.src = 1;	//provider of data is MavLink system in id field
        collision.id = mav_vehicle_->getLocalSystemId();
        collision.action = static_cast<uint8_t>(mavlinkcom::MAV_COLLISION_ACTION::MAV_COLLISION_ACTION_REPORT);
        collision.threat_level = static_cast<uint8_t>(mavlinkcom::MAV_COLLISION_THREAT_LEVEL::MAV_COLLISION_THREAT_LEVEL_NONE);
        // we are abusing these fields, passing the angle of the object we hit, so that jMAVSim knows how to bounce off.
        collision.time_to_minimum_delta = normalX;
        collision.altitude_minimum_delta = normalY;
        collision.horizontal_minimum_delta = normalZ;
        mav_vehicle_->sendMessage(collision);
    }

    bool hasVideoRequest()
    {
        mavlinkcom::MavLinkVideoServer::MavLinkVideoRequest image_req;
        return video_server_->hasVideoRequest(image_req);
    }

    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
    {
        const int MAVLINK_DATA_STREAM_IMG_PNG = 6;
        video_server_->sendFrame(data, length, width, height, MAVLINK_DATA_STREAM_IMG_PNG, 0);
    }

    void setNormalMode()
    {
        if (is_hil_mode_set_ && connection_ != nullptr && mav_vehicle_ != nullptr) {

            // remove MAV_MODE_FLAG_HIL_ENABLED flag from current mode 
            std::lock_guard<std::mutex> guard(set_mode_mutex_);
            int mode = mav_vehicle_->getVehicleState().mode;
            mode &= ~static_cast<int>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_HIL_ENABLED);

            mavlinkcom::MavCmdDoSetMode cmd;
            cmd.command = static_cast<uint16_t>(mavlinkcom::MAV_CMD::MAV_CMD_DO_SET_MODE);
            cmd.Mode = static_cast<float>(mode);
            mav_vehicle_->sendCommand(cmd);

            is_hil_mode_set_ = false;
        }
    }

    void setHILMode()
    {
        if (!is_simulation_mode_)
            throw std::logic_error("Attempt to set device in HIL mode while not in simulation mode");


        checkVehicle();

        // add MAV_MODE_FLAG_HIL_ENABLED flag to current mode 
        std::lock_guard<std::mutex> guard(set_mode_mutex_);
        int mode = mav_vehicle_->getVehicleState().mode;
        mode |= static_cast<int>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_HIL_ENABLED);

        mavlinkcom::MavCmdDoSetMode cmd;
        cmd.command = static_cast<uint16_t>(mavlinkcom::MAV_CMD::MAV_CMD_DO_SET_MODE);
        cmd.Mode = static_cast<float>(mode);
        mav_vehicle_->sendCommand(cmd);

        is_hil_mode_set_ = true;
    }

    void close()
    {
        if (connection_ != nullptr) {
            if (is_hil_mode_set_ && mav_vehicle_ != nullptr) {
                setNormalMode();
            }

            connection_->close();
        }

        if (hil_node_ != nullptr)
            hil_node_->close();

        if (video_server_ != nullptr)
            video_server_->close();

        if (logviewer_proxy_ != nullptr) {
            logviewer_proxy_->close();
            logviewer_proxy_ = nullptr;
        }

        if (logviewer_out_proxy_ != nullptr) {
            if (mav_vehicle_ != nullptr) {
                mav_vehicle_->getConnection()->stopLoggingSendMessage();
            }
            logviewer_out_proxy_->close();
            logviewer_out_proxy_ = nullptr;
        }

        if (qgc_proxy_ != nullptr) {
            qgc_proxy_->close();
            qgc_proxy_ = nullptr;
        }
        if (mav_vehicle_ != nullptr) {
            mav_vehicle_->close();
            mav_vehicle_ = nullptr;
        }
    }

    //additional methods for DroneControllerBase
    void updateState()
    {
        StatusLock lock(parent_);
        if (mav_vehicle_ != nullptr) {
            int version = mav_vehicle_->getVehicleStateVersion();
            if (version != state_version_)
            {
                current_state = mav_vehicle_->getVehicleState();
                state_version_ = version;
            }
        }
    }
    
    Kinematics::State getKinematicsEstimated()
    {
        updateState();
        Kinematics::State state;
        //TODO: reduce code duplication below?
        state.pose.position = Vector3r(current_state.local_est.pos.x, current_state.local_est.pos.y, current_state.local_est.pos.z);
        state.pose.orientation = VectorMath::toQuaternion(current_state.attitude.pitch, current_state.attitude.roll, current_state.attitude.yaw);
        state.twist.linear = Vector3r(current_state.local_est.lin_vel.x, current_state.local_est.lin_vel.y, current_state.local_est.lin_vel.z);
        state.twist.angular = Vector3r(current_state.attitude.roll_rate, current_state.attitude.pitch_rate, current_state.attitude.yaw_rate);
        state.pose.position = Vector3r(current_state.local_est.acc.x, current_state.local_est.acc.y, current_state.local_est.acc.z);
        //TODO: how do we get angular acceleration?
        return state;
    }

    Vector3r getPosition()
    {
        updateState();
        return Vector3r(current_state.local_est.pos.x, current_state.local_est.pos.y, current_state.local_est.pos.z);
    }

    Vector3r getVelocity()
    {
        updateState();
        return Vector3r(current_state.local_est.lin_vel.x, current_state.local_est.lin_vel.y, current_state.local_est.lin_vel.z);
    }

    Quaternionr getOrientation()
    {
        updateState();
        return VectorMath::toQuaternion(current_state.attitude.pitch, current_state.attitude.roll, current_state.attitude.yaw);
    }

    GeoPoint getHomeGeoPoint()
    {
        updateState();
        if (current_state.home.is_set)
            return GeoPoint(current_state.home.global_pos.lat, current_state.home.global_pos.lon, current_state.home.global_pos.alt);
        else
            return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
    }

    GeoPoint getGpsLocation()
    {
        updateState();
        return GeoPoint(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.global_est.pos.alt);
    }


    LandedState getLandedState()
    {
        updateState();
        return current_state.controls.landed ? LandedState::Landed : LandedState::Flying;
    }

    //administrative

    bool armDisarm(bool arm, CancelableBase& cancelable_action)
    {
        unused(arm);
        unused(cancelable_action);
        checkVehicle();
        bool rc = false;
        mav_vehicle_->armDisarm(arm).wait(10000, &rc);
        return rc;
    }

    bool isApiControlEnabled()
    {
        return is_api_control_enabled_;
    }

    bool isSimulationMode()
    {
        return is_simulation_mode_;
    }

    void enableApiControl(bool is_enabled)
    {
        checkVehicle();
        if (is_enabled) {
            mav_vehicle_->requestControl();
            is_api_control_enabled_ = true;
        }
        else {
            mav_vehicle_->releaseControl();
            is_api_control_enabled_ = false;
        }
    }

    void setSimulationMode(bool is_set)
    {
        if (connection_ != nullptr) {
            //TODO: can we do this?
            throw std::domain_error("Cannot call setSimulationMode after connection was already established for MavLink");
        }

        is_simulation_mode_ = is_set;
    }

    bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action)
    {
        unused(cancelable_action);
        checkVehicle();

        bool rc = false;
        auto vec = getPosition();
        float z = vec.z() + getTakeoffZ();
        if (!mav_vehicle_->takeoff(z, 0.0f /* pitch */, 0.0f /* yaw */).wait(static_cast<int>(max_wait_seconds * 1000), &rc))
        {
            throw VehicleMoveException("TakeOff command - timeout waiting for response");
        }
        if (!rc) {
            throw VehicleMoveException("TakeOff command rejected by drone");
        }
        if (max_wait_seconds <= 0)
            return true; // client doesn't want to wait.

        return parent_->waitForZ(max_wait_seconds, z, getDistanceAccuracy(), cancelable_action);
    }

    bool hover(CancelableBase& cancelable_action)
    {
        bool rc = false;
        checkVehicle();
        mavlinkcom::AsyncResult<bool> result = mav_vehicle_->loiter();
        //auto start_time = std::chrono::system_clock::now();
        while (!cancelable_action.isCancelled())
        {
            if (result.wait(100, &rc))
            {
                break;
            }
        }
        return rc;
    }

    bool land(float max_wait_seconds, CancelableBase& cancelable_action)
    {
        unused(cancelable_action);
        // bugbug: really need a downward pointing distance to ground sensor to do this properly, for now
        // we assume the ground is relatively flat an we are landing roughly at the home altitude.
        updateState();
        checkVehicle();
        if (current_state.home.is_set)
        {
            bool rc = false;
            if (!mav_vehicle_->land(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.home.global_pos.alt).wait(10000, &rc))
            {
                throw VehicleMoveException("Landing command - timeout waiting for response from drone");
            }
            else if (!rc) {
                throw VehicleMoveException("Landing command rejected by drone");
            }
        }
        else
        {
            throw VehicleMoveException("Cannot land safely with out a home position that tells us the home altitude.  Could fix this if we hook up a distance to ground sensor...");
        }

        // Wait for landed state (or user cancelation)
        if (!parent_->waitForFunction([&]() {
            updateState();
            return current_state.controls.landed;
        }, max_wait_seconds, cancelable_action))
        {
            throw VehicleMoveException("Drone hasn't reported a landing state");
        }
        return true;
    }

    bool goHome(CancelableBase& cancelable_action)
    {
        unused(cancelable_action);
        checkVehicle();
        bool rc = false;
        if (mav_vehicle_ != nullptr && !mav_vehicle_->returnToHome().wait(10000, &rc)) {
            throw VehicleMoveException("goHome - timeout waiting for response from drone");
        }
        return rc;
    }

    void commandRollPitchZ(float pitch, float roll, float z, float yaw)
    {
        if (target_height_ != -z) {
            // these PID values were calculated experimentally using AltHoldCommand n MavLinkTest, this provides the best
            // control over thrust to achieve minimal over/under shoot in a reasonable amount of time, but it has not
            // been tested on a real drone outside jMavSim, so it may need recalibrating...
            thrust_controller_.setPoint(-z, .05f, .005f, 0.09f);
            target_height_ = -z;
        }
        checkVehicle();
        auto state = mav_vehicle_->getVehicleState();
        float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
        mav_vehicle_->moveByAttitude(roll, pitch, yaw, 0, 0, 0, thrust);
    }
    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
    {
        checkVehicle();
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        mav_vehicle_->moveByLocalVelocity(vx, vy, vz, !yaw_mode.is_rate, yaw);
    }
    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
    {
        checkVehicle();
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        mav_vehicle_->moveByLocalVelocityWithAltHold(vx, vy, z, !yaw_mode.is_rate, yaw);
    }
    void commandPosition(float x, float y, float z, const YawMode& yaw_mode)
    {
        checkVehicle();
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        mav_vehicle_->moveToLocalPosition(x, y, z, !yaw_mode.is_rate, yaw);
    }

    RCData getRCData()
    {
        throw VehicleCommandNotImplementedException("getRCData() function is not yet implemented");
    }

    void setRCData(const RCData& rcData)
    {
        unused(rcData);
        //TODO: use RC data to control MavLink drone
    }

    bool validateRCData(const RCData& rc_data)
    {
        unused(rc_data);
        return true;
    }

    //drone parameters
    float getCommandPeriod()
    {
        return 1.0f / 50; //1 period of 50hz
    }
    float getTakeoffZ()
    {
        // pick a number, PX4 doesn't have a fixed limit here, but 3 meters is probably safe 
        // enough to get out of the backwash turbulance.  Negative due to NED coordinate system.
        return -3.0f;
    }
    float getDistanceAccuracy()
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
    }
    const VehicleParams& getVehicleParams()
    {
        return getInternalVehicleParams(); //defaults are good for PX4 generic quadrocopter.
    }
    //TODO: decouple DroneControllerBase, VehicalParams and SafetyEval
    const VehicleParams& getInternalVehicleParams()
    {
        static const VehicleParams vehicle_params_;
        return vehicle_params_; //defaults are good for DJI Matrice 100
    }

    void reportTelemetry(float renderTime)
    {
        if (logviewer_proxy_ == nullptr || connection_ == nullptr || mav_vehicle_ == nullptr) {
            return;
        }
        mavlinkcom::MavLinkTelemetry data;
        connection_->getTelemetry(data);
        if (data.messagesReceived == 0) {
            if (!hil_message_timer_.started()) {
                hil_message_timer_.start();
            } else if (hil_message_timer_.seconds() > messageReceivedTimeout) {
                addStatusMessage("not receiving any messages from HIL, please restart your HIL node and try again");
            }
        } else {
            hil_message_timer_.stop();
        }

        // listen to the other mavlink connection also
        auto mavcon = mav_vehicle_->getConnection();
        if (mavcon != connection_) {
            mavlinkcom::MavLinkTelemetry sitl;
            mavcon->getTelemetry(sitl);

            data.handlerMicroseconds += sitl.handlerMicroseconds;
            data.messagesHandled += sitl.messagesHandled;
            data.messagesReceived += sitl.messagesReceived;
            data.messagesSent += sitl.messagesSent;

            if (sitl.messagesReceived == 0)
            {
                if (!sitl_message_timer_.started()) {
                    sitl_message_timer_.start();
                }
                else if (sitl_message_timer_.seconds() > messageReceivedTimeout) {
                    addStatusMessage("not receiving any messages from SITL, please restart your SITL node and try again");
                }
            }
            else {
                sitl_message_timer_.stop();
            }
        }

        data.renderTime = static_cast<int64_t>(renderTime * 1000000);// microseconds
        logviewer_proxy_->sendMessage(data);
    }

    Pose getDebugPose()
    {
        std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
        return debug_pose_;
    }

    bool startOffboardMode()
    {
        checkVehicle();
        try {
            mav_vehicle_->requestControl();
        }
        catch (std::exception& ex) {
            ensureSafeMode();
            addStatusMessage(std::string("Request control failed: ") + ex.what());
            return false;
        }
        return true;
    }

    void endOffboardMode()
    {
        // bugbug: I removed this releaseControl because it makes back-to-back move operations less smooth.
        // The side effect of this is that with some drones (e.g. PX4 based) the drone itself will timeout
        // when you stop sending move commands and the behavior on timeout is then determined by the drone itself.
        // mav_vehicle_->releaseControl();
        ensureSafeMode();
    }

    void ensureSafeMode()
    {
        if (mav_vehicle_ != nullptr) {
            const mavlinkcom::VehicleState& state = mav_vehicle_->getVehicleState();
            if (state.controls.landed || !state.controls.armed) {
                return;
            }
        }
    }

    bool loopCommandPre()
    {
        return startOffboardMode();
    }

    void loopCommandPost()
    {
        endOffboardMode();
    }

    void checkVehicle() {
        if (mav_vehicle_ == nullptr) {
            throw std::logic_error("Cannot perform operation when no vehicle is connected");
        }
    }
}; //impl

   //empty constructor required for pimpl
MavLinkDroneController::MavLinkDroneController()
{
    pimpl_.reset(new impl(this));
}

MavLinkDroneController::~MavLinkDroneController()
{
    pimpl_->closeAllConnection();
}

void MavLinkDroneController::initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation)
{
    pimpl_->initialize(connection_info, sensors, is_simulation);
}

MavLinkDroneController::ConnectionInfo MavLinkDroneController::getMavConnectionInfo()
{
    return pimpl_->getMavConnectionInfo();
}
void MavLinkDroneController::sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
{
    pimpl_->sendImage(data, length, width, height);
}

bool MavLinkDroneController::hasVideoRequest()
{
    return pimpl_->hasVideoRequest();
}
std::string MavLinkDroneController::findPX4()
{
    return impl::findPX4();
}


//*** Start: VehicleControllerBase implementation ***//
void MavLinkDroneController::reset()
{
    DroneControllerBase::reset();
    pimpl_->reset();
}
void MavLinkDroneController::update()
{
    DroneControllerBase::update();
    pimpl_->update();
}
real_T MavLinkDroneController::getVertexControlSignal(unsigned int rotor_index)
{
    return pimpl_->getVertexControlSignal(rotor_index);
}
size_t MavLinkDroneController::getVertexCount()
{
    return impl::RotorControlsCount;
}
void MavLinkDroneController::getStatusMessages(std::vector<std::string>& messages)
{
    pimpl_->getStatusMessages(messages);
}
bool MavLinkDroneController::isAvailable(std::string& message)
{
    return pimpl_->isAvailable(message);
}

//*** End: VehicleControllerBase implementation ***//



//DroneControlBase
Kinematics::State MavLinkDroneController::getKinematicsEstimated()
{
    return pimpl_->getKinematicsEstimated();
}

Vector3r MavLinkDroneController::getPosition()
{
    return pimpl_->getPosition();
}

Vector3r MavLinkDroneController::getVelocity()
{
    return pimpl_->getVelocity();
}

Quaternionr MavLinkDroneController::getOrientation()
{
    return pimpl_->getOrientation();
}

GeoPoint MavLinkDroneController::getHomeGeoPoint()
{
    return pimpl_->getHomeGeoPoint();
}

GeoPoint MavLinkDroneController::getGpsLocation()
{
    return pimpl_->getGpsLocation();
}

DroneControllerBase::LandedState MavLinkDroneController::getLandedState()
{
    return pimpl_->getLandedState();
}
//administrative

bool MavLinkDroneController::armDisarm(bool arm, CancelableBase& cancelable_action)
{
    return pimpl_->armDisarm(arm, cancelable_action);
}


void MavLinkDroneController::enableApiControl(bool is_enabled)
{
    pimpl_->enableApiControl(is_enabled);
}
void MavLinkDroneController::setSimulationMode(bool is_set)
{
    pimpl_->setSimulationMode(is_set);
}
bool MavLinkDroneController::isApiControlEnabled()
{
    return pimpl_->isApiControlEnabled();
}
bool MavLinkDroneController::isSimulationMode()
{
    return pimpl_->isSimulationMode();
}

bool MavLinkDroneController::takeoff(float max_wait_seconds, CancelableBase& cancelable_action)
{
    return pimpl_->takeoff(max_wait_seconds, cancelable_action);
}

bool MavLinkDroneController::hover(CancelableBase& cancelable_action)
{
    return pimpl_->hover(cancelable_action);
}

bool MavLinkDroneController::land(float max_wait_seconds, CancelableBase& cancelable_action)
{
    return pimpl_->land(max_wait_seconds, cancelable_action);
}

bool MavLinkDroneController::goHome(CancelableBase& cancelable_action)
{
    return pimpl_->goHome(cancelable_action);
}

void MavLinkDroneController::commandRollPitchZ(float pitch, float roll, float z, float yaw)
{
    return pimpl_->commandRollPitchZ(pitch, roll, z, yaw);
}
void MavLinkDroneController::commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
{
    return pimpl_->commandVelocity(vx, vy, vz, yaw_mode);
}
void MavLinkDroneController::commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
{
    return pimpl_->commandVelocityZ(vx, vy, z, yaw_mode);
}
void MavLinkDroneController::commandPosition(float x, float y, float z, const YawMode& yaw_mode)
{
    return pimpl_->commandPosition(x, y, z, yaw_mode);
}

RCData MavLinkDroneController::getRCData()
{
    return pimpl_->getRCData();
}
void MavLinkDroneController::setRCData(const RCData& rcData)
{
    return pimpl_->setRCData(rcData);
}

bool MavLinkDroneController::loopCommandPre()
{
    return pimpl_->loopCommandPre();
}

void MavLinkDroneController::loopCommandPost()
{
    pimpl_->loopCommandPost();
}

//drone parameters
float MavLinkDroneController::getCommandPeriod()
{
    return pimpl_->getCommandPeriod();
}
float MavLinkDroneController::getTakeoffZ()
{
    return pimpl_->getTakeoffZ();
}
float MavLinkDroneController::getDistanceAccuracy()
{
    return pimpl_->getDistanceAccuracy();
}
const VehicleParams& MavLinkDroneController::getVehicleParams()
{
    return pimpl_->getVehicleParams();
}
//TODO: decouple DroneControllerBase, VehicalParams and SafetyEval

void MavLinkDroneController::reportTelemetry(float renderTime)
{
    return pimpl_->reportTelemetry(renderTime);
}

Pose MavLinkDroneController::getDebugPose()
{
    return pimpl_->getDebugPose();
}



}} //namespace
#endif

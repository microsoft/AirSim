// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "controllers/MavLinkDroneController.hpp"
#include <memory>
#include <exception>
#include "controllers/PidController.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkNode.hpp"
#include "MavLinkVideoStream.hpp"
#include "MavLinkVehicle.hpp"

//sensors
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"

namespace msr { namespace airlib {


using namespace mavlinkcom;

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board
static const int RotorControlsCount = 8;

struct MavLinkDroneController::impl {
    std::shared_ptr<mavlinkcom::MavLinkNode> logviewer_proxy_, qgc_proxy_;

    size_t status_messages_MaxSize = 5000;
    
    std::shared_ptr<mavlinkcom::MavLinkNode> main_node_;
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
    mavlinkcom::MavLinkHilStateQuaternion HilStateQuaternionMessage;

    mavlinkcom::MavLinkHilSensor last_sensor_message_;
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
    long last_gps_time_;
    bool was_reseted_;

    //additional variables required for DroneControllerBase implementation
    //this is optional for methods that might not use vehicle commands
    std::unique_ptr<mavlinkcom::MavLinkVehicle> mav_vehicle_;
    int state_version_;
    mavlinkcom::VehicleState current_state;
    float target_height_;
    bool is_offboard_mode_;
    bool is_simulation_mode_;
    PidController thrust_controller_;

    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation)
    {
        connection_info_ = connection_info;
        sensors_ = sensors;
        is_simulation_mode_ = is_simulation;

        mav_vehicle_.reset(new mavlinkcom::MavLinkVehicle(connection_info_.vehicle_sysid, connection_info_.vehicle_compid));
    }

    ConnectionInfo getMavConnectionInfo()
    {
        return connection_info_;
    }

    void normalizeRotorControls()
    {
        if (!is_controls_0_1_) {
            // change -1 to 1 to 0 to 1.
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = (rotor_controls_[i] + 1.0f) / 2.0f;
            }
        }
        else { // we have 0 to 1
            //TODO: make normalization vehicle independent?
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = Utils::clip(0.83f * rotor_controls_[i] + 0.17f, 0.0f, 1.0f);
            }
        }
    }

    void initializeMavSubscrptions()
    {
        if (connection_ != nullptr) {
            is_any_heartbeat_ = false;
            is_hil_mode_set_ = false;
            is_armed_ = false;
            is_controls_0_1_ = true;
            Utils::setValue(rotor_controls_, 0.0f);
            //TODO: main_node_->setMessageInterval(...);
            auto subscriber = std::bind(&impl::hILSubscriber, this, std::placeholders::_1, std::placeholders::_2);
            connection_->subscribe(subscriber);
        }
    }

    bool sendTestMessage(std::shared_ptr<MavLinkNode> node) {
        try {
            // try and send a test message.
            MavLinkHeartbeat test;
            test.autopilot = static_cast<int>(MAV_AUTOPILOT::MAV_AUTOPILOT_PX4);
            test.type = static_cast<uint8_t>(MAV_TYPE::MAV_TYPE_GCS);
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
            logviewer_proxy_ = createProxy("LogViewer", connection_info_.logviewer_ip_address, connection_info_.logviewer_ip_port, connection_info_.local_host_ip);
            if (!sendTestMessage(logviewer_proxy_)) {
                // error talking to log viewer, so don't keep trying, and close the connection also.
                logviewer_proxy_->getConnection()->close();
                logviewer_proxy_ = nullptr;
            }
        }
        return logviewer_proxy_ != nullptr;
    }

    bool connectToQGC()
    {
        if (connection_info_.qgc_ip_address.size() > 0) {
            qgc_proxy_ = createProxy("QGC", connection_info_.qgc_ip_address, connection_info_.qgc_ip_port, connection_info_.local_host_ip);
            if (!sendTestMessage(qgc_proxy_)) {
                // error talking to QGC, so don't keep trying, and close the connection also.
                qgc_proxy_->getConnection()->close();
                qgc_proxy_ = nullptr;
            }
        }
        return qgc_proxy_ != nullptr;
    }


    std::shared_ptr<mavlinkcom::MavLinkNode> createProxy(std::string name, std::string ip, int port, string local_host_ip)
    {
        if (connection_ == nullptr)
            throw std::domain_error("MavLinkDroneController requires connection object to be set before createProxy call");

        auto connection = MavLinkConnection::connectRemoteUdp("Proxy to: " + name + " at " + ip + ":" + std::to_string(port), local_host_ip, ip, port);

        // it is ok to reuse the simulator sysid and compid here because this node is only used to send a few messages directly to this endpoint
        // and all other messages are funnelled through from PX4 via the Join method below.
        auto node = std::make_shared<MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        node->connect(connection);

        // now join the main connection to this one, this causes all PX4 messages to be sent to the proxy and all messages from the proxy will be
        // send directly to the PX4 (using whatever sysid/compid comes from that remote node).
        connection_->join(connection);

        return node;
    }

    static std::string findPixhawk()
    {
        auto result = MavLinkConnection::findSerialPorts(0, 0);
        for (auto iter = result.begin(); iter != result.end(); iter++)
        {
            SerialPortInfo info = *iter;
            if (info.vid == pixhawkVendorId) {
                if (info.pid == pixhawkFMUV4ProductId || info.pid == pixhawkFMUV2ProductId || info.pid == pixhawkFMUV2OldBootloaderProductId)
                {
                    // printf("Auto Selecting COM port: %S\n", info.displayName.c_str());
                    return std::string(info.portName.begin(), info.portName.end());
                }
            }
        }
        return "";
    }

    void connect()
    {
        createMavConnection(connection_info_);
        initializeMavSubscrptions();
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
            throw std::invalid_argument("mav_vehicle_udp parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/mav_vehicle_udp' to the local UDP address to start listening on for the mavlink messages, for example: '127.0.0.1' or '192.168.1.64'.\n");
        }

        if (port == 0) {
            throw std::invalid_argument("mav_vehicle_udp_port parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/mav_vehicle_udp_port' to to the local UDP port start listening on for mavlink messages, for example: 14550.\n");
        }

        connection_ = MavLinkConnection::connectRemoteUdp("hil", connection_info_.local_host_ip, ip, port);
        main_node_ = std::make_shared<MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid); 
        main_node_->connect(connection_);
    }

    void createMavSerialConnection(const std::string& port_name, int baud_rate)
    {
        close();

        std::string port_name_auto = port_name;
        if (port_name_auto == "" || port_name_auto == "*") {
            port_name_auto = findPixhawk();
            if (port_name_auto == "") {
                throw std::domain_error("Could not find a connected PX4 flight controller");
            }
        }

        if (port_name_auto == "") {
            throw std::invalid_argument("mav_vehicle_serial parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/mav_vehicle_serial' to the name of the serial device, for example '/dev/ttyACM0'.  Note you can ls -l /dev/serial/by-id to find which device your PX4 is showing up as.\n");
        }

        if (baud_rate == 0) {
            throw std::invalid_argument("mav_vehicle_params.baudRate parameter is not set, or has an invalid value.  Please run 'rosparam' to set '/AirNode/mav_vehicle_params.baudRate' to to the correct value, for example: 115200 or 57600\n");
        }

        connection_ = MavLinkConnection::connectSerial("hil", port_name_auto, baud_rate);
        main_node_ = std::make_shared<MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
        main_node_->connect(connection_);
    }

    mavlinkcom::MavLinkHilSensor getLastSensorMessage()
    {
        std::lock_guard<std::mutex> guard(last_message_mutex_);
        return last_sensor_message_;
    }

    mavlinkcom::MavLinkHilGps getLastGpsMessage()
    {
        std::lock_guard<std::mutex> guard(last_message_mutex_);
        return last_gps_message_;
    }

    void externalSimSubscriber(std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg)
    {
        if (msg.msgid == MocapPoseMessage.msgid) {
            std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
            MocapPoseMessage.decode(msg); // update current vehicle state.
        }
    }

    void hILSubscriber(std::shared_ptr<MavLinkConnection> connection, const MavLinkMessage& msg)
    {
        processMavMessages(msg);
    }

    void processMavMessages(const MavLinkMessage& msg)
    {
        if (msg.msgid == HeartbeatMessage.msgid) {
            std::lock_guard<std::mutex> guard_heartbeat(heartbeat_mutex_);

            //TODO: have MavLinkNode track armed state so we don't have to re-decode message here again
            HeartbeatMessage.decode(msg);
            is_armed_ = (HeartbeatMessage.base_mode & static_cast<uint8_t>(MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED)) > 0;
            if (!is_any_heartbeat_) {
                is_any_heartbeat_ = true;
                if (HeartbeatMessage.autopilot == static_cast<uint8_t>(MAV_AUTOPILOT::MAV_AUTOPILOT_PX4) &&
                    HeartbeatMessage.type == static_cast<uint8_t>(MAV_TYPE::MAV_TYPE_FIXED_WING)) {
                    // PX4 will scale fixed wing servo outputs to -1 to 1
                    // and it scales multi rotor servo outpus to 0 to 1.
                    is_controls_0_1_ = false;
                }
            } else if (is_simulation_mode_ && !is_hil_mode_set_) {
                setHILMode();
            }
        } else if (msg.msgid == StatusTextMessage.msgid) {
            std::lock_guard<std::mutex> guard_status(status_text_mutex_);
            StatusTextMessage.decode(msg);
            //if queue became too large, clear it first
            if (status_messages_.size() > status_messages_MaxSize)
                Utils::clear(status_messages_, status_messages_MaxSize - status_messages_.size());
            status_messages_.push(std::string(StatusTextMessage.text));
        } else if (msg.msgid == CommandLongMessage.msgid) {
            CommandLongMessage.decode(msg);
            if (CommandLongMessage.command == static_cast<int>(MAV_CMD::MAV_CMD_SET_MESSAGE_INTERVAL)) {
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
    }

    void sendHILSensor(const Vector3r& acceleration, const Vector3r& gyro, const Vector3r& mag, float abs_pressure, float pressure_alt)
    {
        mavlinkcom::MavLinkHilSensor hil_sensor;
        hil_sensor.time_usec = static_cast<uint64_t>(Utils::getTimeSinceEpochMillis() * 1000);
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
        //TODO: enable tempeprature? diff_presure
        hil_sensor.fields_updated = was_reseted_ ? (1 << 31) : 0;

        if (main_node_ != nullptr) {
            main_node_->sendMessage(hil_sensor);
        }

        // also forward this message to the log viewer so we can see it there also.
        if (logviewer_proxy_ != nullptr) {
            logviewer_proxy_->sendMessage(hil_sensor);
        }

        std::lock_guard<std::mutex> guard(last_message_mutex_);
        last_sensor_message_ = hil_sensor;
    }

    void sendHILGps(const GeoPoint& geo_point, const Vector3r& velocity, float velocity_xy, float cog,
        float eph, float epv, int fix_type, unsigned int satellites_visible)
    {
        mavlinkcom::MavLinkHilGps hil_gps;
        hil_gps.time_usec = static_cast<uint64_t>(Utils::getTimeSinceEpochMillis() * 1000);
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

        if (main_node_ != nullptr) {
            main_node_->sendMessage(hil_gps);
        }

        if (logviewer_proxy_ != nullptr) {
            logviewer_proxy_->sendMessage(hil_gps);
        }

        std::lock_guard<std::mutex> guard(last_message_mutex_);
        last_gps_message_ = hil_gps;
    }

    real_T getVertexControlSignal(unsigned int rotor_index)
    {
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
        is_offboard_mode_ = false;
        is_simulation_mode_ = false;
        thrust_controller_ = PidController();
        Utils::setValue(rotor_controls_, 0.0f);
        was_reseted_ = false;
    }

    //*** Start: VehicleControllerBase implementation ***//
    void reset()
    {
        resetState();
        was_reseted_ = true;
        setNormalMode();
    }

    const ImuBase* getImu()
    {
        return static_cast<const ImuBase*>(sensors_->getByType(SensorCollection::SensorType::Imu));
    }
    const MagnetometerBase* getMagnetometer()
    {
        return static_cast<const MagnetometerBase*>(sensors_->getByType(SensorCollection::SensorType::Magnetometer));
    }
    const BarometerBase* getBarometer()
    {
        return static_cast<const BarometerBase*>(sensors_->getByType(SensorCollection::SensorType::Barometer));
    }
    const GpsBase* getGps()
    {
        return static_cast<const GpsBase*>(sensors_->getByType(SensorCollection::SensorType::Gps));
    }

    void update(real_T dt)
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
        if (was_reseted_)
            was_reseted_ = false;
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

    void start()
    {
        close();
        resetState();

        connect();
        connectToLogViewer();
        connectToQGC();

        mav_vehicle_->connect(connection_);
    }
    void stop()
    {
        close();
    }
    //*** End: VehicleControllerBase implementation ***//

    void getMocapPose(Vector3r& position, Quaternionr& orientation)
    {
        std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
        position.x() = MocapPoseMessage.x; position.y() = MocapPoseMessage.y; position.z() = MocapPoseMessage.z; 
        orientation.w() = MocapPoseMessage.q[0]; orientation.x() = MocapPoseMessage.q[1]; 
        orientation.y() = MocapPoseMessage.q[2]; orientation.z() = MocapPoseMessage.q[3];
    }

    void sendMocapPose(const Vector3r& position, const Quaternionr& orientation)
    {
        if (main_node_ == nullptr) return;

        mavlinkcom::MavLinkAttPosMocap mocap_pose_message;
        mocap_pose_message.x = position.x(); mocap_pose_message.y = position.y(); mocap_pose_message.z = position.z();
        mocap_pose_message.q[0] = orientation.w(); mocap_pose_message.q[1] = orientation.x();
        mocap_pose_message.q[2] = orientation.y(); mocap_pose_message.q[3] = orientation.z();
        main_node_->sendMessage(mocap_pose_message);	
    }

    void sendCollison(float normalX, float normalY, float normalZ)
    {
        if (main_node_ == nullptr) return;

        MavLinkCollision collision{};
        collision.src = 1;	//provider of data is MavLink system in id field
        collision.id = main_node_->getLocalSystemId();
        collision.action = static_cast<uint8_t>(MAV_COLLISION_ACTION::MAV_COLLISION_ACTION_REPORT);
        collision.threat_level = static_cast<uint8_t>(MAV_COLLISION_THREAT_LEVEL::MAV_COLLISION_THREAT_LEVEL_NONE);
        // we are abusing these fields, passing the angle of the object we hit, so that jMAVSim knows how to bounce off.
        collision.time_to_minimum_delta = normalX;
        collision.altitude_minimum_delta = normalY;
        collision.horizontal_minimum_delta = normalZ;
        main_node_->sendMessage(collision);
    }

    bool hasVideoRequest()
    {
        MavLinkVideoServer::MavLinkVideoRequest image_req;
        return video_server_->hasVideoRequest(image_req);
    }

    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
    {
        const int MAVLINK_DATA_STREAM_IMG_PNG = 6;
        video_server_->sendFrame(data, length, width, height, MAVLINK_DATA_STREAM_IMG_PNG, 0);
    }

    void setNormalMode()
    {
        if (is_hil_mode_set_ && main_node_ != nullptr && connection_ != nullptr) {
            //TODO: this is depricated message, add support for MAV_CMD_DO_SET_MODE
            std::lock_guard<std::mutex> guard(set_mode_mutex_);
            SetModeMessage.target_system = connection_info_.sim_sysid;
            SetModeMessage.base_mode = 0;  //disarm
            main_node_->sendMessage(SetModeMessage);

            mavlinkcom::MavCmdComponentArmDisarm disarm_msg;
            disarm_msg.p1ToArm = 0;
            main_node_->sendCommand(disarm_msg);

            is_hil_mode_set_ = false;
        }
    }

    void setHILMode()
    {
        if (main_node_ == nullptr) {
            return;
        }

        //TODO: this is depricated message, add support for MAV_CMD_DO_SET_MODE
        std::lock_guard<std::mutex> guard_setmode(set_mode_mutex_);
        SetModeMessage.target_system = connection_info_.sim_sysid;
        SetModeMessage.base_mode = 32;  //HIL + disarm
        main_node_->sendMessage(SetModeMessage);
        is_hil_mode_set_ = true;
    }

    void close()
    {
        if (connection_ != nullptr) {
            if (is_hil_mode_set_ && main_node_ != nullptr) {
                setNormalMode();
            }

            connection_->close();
        }

        if (main_node_ != nullptr)
            main_node_->close();

        if (video_server_ != nullptr)
            video_server_->close();

        if (logviewer_proxy_ != nullptr) {
            logviewer_proxy_->close();
            logviewer_proxy_ = nullptr;
        }
        if (qgc_proxy_ != nullptr) {
            qgc_proxy_->close();
            qgc_proxy_ = nullptr;
        }
        if (mav_vehicle_ != nullptr) {
            mav_vehicle_->close();
            //do not remove reference otherwise we need to recreate when restarting
            //mav_vehicle_ = nullptr;
        }
    }

    //additional methods for DroneControllerBase
    void updateState()
    {
        StatusLock lock(parent_);
        int version = mav_vehicle_->getVehicleStateVersion();
        if (version != state_version_)
        {
            current_state = mav_vehicle_->getVehicleState();
            state_version_ = version;
        }
    }

    Vector3r getPosition()
    {
        updateState();
        return Vector3r(current_state.local_est.pos.x, current_state.local_est.pos.y, current_state.local_est.pos.z);
    }

    Vector3r getVelocity()
    {
        updateState();
        return Vector3r(current_state.local_est.vel.vx, current_state.local_est.vel.vy, current_state.local_est.vel.vz);
    }

    GeoPoint getHomePoint()
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

    Quaternionr getOrientation()
    {
        updateState();
        return VectorMath::toQuaternion(current_state.attitude.pitch, current_state.attitude.roll, current_state.attitude.yaw);
    }

    double timestampNow()
    {
        return static_cast<double>(mav_vehicle_->getTimeStamp());
    }

    //administrative

    bool armDisarm(bool arm, CancelableBase& cancelable_action)
    {
        bool rc = false;
        mav_vehicle_->armDisarm(arm).wait(10000, &rc);
        return rc;
    }

    bool isOffboardMode()
    {
        return is_offboard_mode_;
    }

    bool isSimulationMode()
    {
        return is_simulation_mode_;
    }

    void setUserInputs(const vector<float>& inputs)
    {
        //TODO: support XBox/Keyboard inputs for MavLink based drone?
    }

    void setOffboardMode(bool is_set)
    {
        if (is_set) {
            mav_vehicle_->requestControl();
            is_offboard_mode_ = true;
        }
        else {
            mav_vehicle_->releaseControl();
            is_offboard_mode_ = false;
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
        bool rc = false;
        if (!mav_vehicle_->takeoff(getTakeoffZ(), 0.0f, 0.0f).wait(static_cast<int>(max_wait_seconds * 1000), &rc))
        {
            throw VehicleMoveException("TakeOff command - timeout waiting for response");
        }
        if (!rc) {
            throw VehicleMoveException("TakeOff command rejected by drone");
        }

        bool success = parent_->waitForZ(max_wait_seconds, getTakeoffZ(), getDistanceAccuracy(), cancelable_action);
        return success;
    }

    bool hover(CancelableBase& cancelable_action)
    {
        bool rc = false;
        AsyncResult<bool> result = mav_vehicle_->loiter();
        auto start_time = std::chrono::system_clock::now();
        while (!cancelable_action.isCancelled())
        {
            if (result.wait(100, &rc))
            {
                break;
            }
        }
        return rc;
    }

    bool land(CancelableBase& cancelable_action)
    {
        // bugbug: really need a downward pointing distance to ground sensor to do this properly, for now
        // we assume the ground is relatively flat an we are landing roughly at the home altitude.
        updateState();
        if (current_state.home.is_set)
        {
            bool rc = false;
            if (!mav_vehicle_->land(current_state.global_est.pos.lat, current_state.global_est.pos.lon, current_state.home.global_pos.alt).wait(10000, &rc))
            {
                throw VehicleMoveException("Landing command - timeout waiting for response from drone");
            }
            else if(!rc) {
                throw VehicleMoveException("Landing command rejected by drone");
            }
        }
        else 
        {
            throw VehicleMoveException("Cannot land safely with out a home position that tells us the home altitude.  Could fix this if we hook up a distance to ground sensor...");
        }

        float max_wait = 60;
        if (!parent_->waitForFunction([&]() {
            updateState();
            return current_state.controls.landed;
        }, max_wait, cancelable_action))
        {
            throw VehicleMoveException("Drone hasn't reported a landing state");
        }
        return true;
    }

    bool goHome(CancelableBase& cancelable_action)
    {
        bool rc = false;
        if (!mav_vehicle_->returnToHome().wait(10000, &rc)) {
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
        auto state = mav_vehicle_->getVehicleState();
        float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
        mav_vehicle_->moveByAttitude(roll, pitch, yaw, 0, 0, 0, thrust);
    }
    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode)
    {
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        mav_vehicle_->moveByLocalVelocity(vx, vy, vz, !yaw_mode.is_rate, yaw);
    }
    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode)
    {
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        mav_vehicle_->moveByLocalVelocityWithAltHold(vx, vy, z, !yaw_mode.is_rate, yaw);
    }
    void commandPosition(float x, float y, float z, const YawMode& yaw_mode)
    {
        float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
        mav_vehicle_->moveToLocalPosition(x, y, z, !yaw_mode.is_rate, yaw);
    }

    //virtual RC mode
    RCData getRCData()
    {
        RCData rc_data;
        return rc_data;
    }

    bool validateRCData(const RCData& rc_data)
    {
        return true;
    }

    void commandVirtualRC(const RCData& rc_data)
    {
        throw VehicleMoveException("commandVirtualRC is not implemented yet");
    }
    void commandEnableVirtualRC(bool enable)
    {
        throw VehicleMoveException("commandVirtualRC is not implemented yet");
    }

    //drone parameters
    float getCommandPeriod() 
    {
        return 1.0f/50; //1 period of 50hz
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
		if (logviewer_proxy_ == nullptr || connection_ == nullptr) {
			return;
		}
		MavLinkTelemetry data;
		connection_->getTelemetry(data);
		data.renderTime = static_cast<long>(renderTime * 1000000);// microseconds
		logviewer_proxy_->sendMessage(data);
	}

}; //impl

//empty constructor required for pimpl
MavLinkDroneController::MavLinkDroneController()
{
    pimpl_.reset(new impl(this));
}

MavLinkDroneController::~MavLinkDroneController()
{
    pimpl_->close();
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
void MavLinkDroneController::getMocapPose(Vector3r& position, Quaternionr& orientation)
{
    pimpl_->getMocapPose(position, orientation);
}
void MavLinkDroneController::sendMocapPose(const Vector3r& position, const Quaternionr& orientation)
{
    pimpl_->sendMocapPose(position, orientation);
}
bool MavLinkDroneController::hasVideoRequest()
{
    return pimpl_->hasVideoRequest();
}
std::string MavLinkDroneController::findPixhawk()
{
    return impl::findPixhawk();
}


//*** Start: VehicleControllerBase implementation ***//
void MavLinkDroneController::reset()
{
    pimpl_->reset();
}
void MavLinkDroneController::update(real_T dt)
{
    pimpl_->update(dt);
}
real_T MavLinkDroneController::getVertexControlSignal(unsigned int rotor_index)
{
    return pimpl_->getVertexControlSignal(rotor_index);
}
size_t MavLinkDroneController::getVertexCount()
{
    return RotorControlsCount;
}
void MavLinkDroneController::start()
{
    pimpl_->start();
}
void MavLinkDroneController::stop()
{
    pimpl_->stop();
}
void MavLinkDroneController::getStatusMessages(std::vector<std::string>& messages)
{
    pimpl_->getStatusMessages(messages);
}
//*** End: VehicleControllerBase implementation ***//



//DroneControlBase
Vector3r MavLinkDroneController::getPosition()
{
    return pimpl_->getPosition();
}

Vector3r MavLinkDroneController::getVelocity()
{
    return pimpl_->getVelocity();
}

GeoPoint MavLinkDroneController::getHomePoint()
{
    return pimpl_->getHomePoint();
}

GeoPoint MavLinkDroneController::getGpsLocation()
{
    return pimpl_->getGpsLocation();
}

Quaternionr MavLinkDroneController::getOrientation()
{
    return pimpl_->getOrientation();
}

double MavLinkDroneController::timestampNow()
{
    return pimpl_->timestampNow();
}

//administrative

bool MavLinkDroneController::armDisarm(bool arm, CancelableBase& cancelable_action)
{
    return pimpl_->armDisarm(arm, cancelable_action);
}


void MavLinkDroneController::setUserInputs(const vector<float>& inputs)
{
    pimpl_->setUserInputs(inputs);
}
void MavLinkDroneController::setOffboardMode(bool is_set)
{
    pimpl_->setOffboardMode(is_set);
}
void MavLinkDroneController::setSimulationMode(bool is_set)
{
    pimpl_->setSimulationMode(is_set);
}
bool MavLinkDroneController::isOffboardMode()
{
    return pimpl_->isOffboardMode();
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

bool MavLinkDroneController::land(CancelableBase& cancelable_action)
{
    return pimpl_->land(cancelable_action);
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

//virtual RC mode
RCData MavLinkDroneController::getRCData()
{
    return pimpl_->getRCData();
}

void MavLinkDroneController::commandVirtualRC(const RCData& rc_data)
{
    return pimpl_->commandVirtualRC(rc_data);
}
void MavLinkDroneController::commandEnableVirtualRC(bool enable)
{
    return pimpl_->commandEnableVirtualRC(enable);
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

}} //namespace
#endif

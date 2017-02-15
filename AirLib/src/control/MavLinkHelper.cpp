// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "control/MavLinkHelper.h"
#include <memory>
#include <exception>

#include "control/MavLinkDroneControl.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkNode.hpp"
#include "MavLinkVideoStream.hpp"

namespace msr { namespace airlib {


using namespace mavlinkcom;

static const int pixhawkVendorId = 9900;   ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
static const int pixhawkFMUV4ProductId = 18;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2ProductId = 17;     ///< Product ID for Pixhawk V2 board
static const int pixhawkFMUV2OldBootloaderProductId = 22;     ///< Product ID for Bootloader on older Pixhawk V2 boards
static const int pixhawkFMUV1ProductId = 16;     ///< Product ID for PX4 FMU V1 board
static const int RotorControlsCount = 8;

struct MavLinkHelper::impl {
    std::shared_ptr<mavlinkcom::MavLinkNode> logviewer_proxy_, qgc_proxy_;

    // mavlink vehicle identifiers
    uint8_t SimSysID = 142;
    int SimCompID = 42;
    uint8_t ExtRendererSysID = 167;
    int ExtRendererCompID = 1;
    uint8_t AirControlSysID = 134;
    int AirControlCompID = 1;

    // if you want to select a specific local network adapter so you can reach certain remote machines (e.g. wifi versus ethernet) 
    // then you will want to change the LocalHostIp accordingly.  This default only works when log viewer and QGC are also on the
    // same machine.  Whatever network you choose it has to be the same one for external
    std::string LocalHostIp = "127.0.0.1";

    // This publishes the sim messagesd from your local machine so others can listen.  It publishes on LocalHostIp. 
    int ExternalSimPort = 14588;

    // The log viewer can be on a different machine, so you can configure it's ip address and port here.
    int LogViewerPort = 14388;
    std::string LogViewerHostIp = "127.0.0.1";

    // The QGroundControl app can be on a different machine, so you can configure it's ip address and port here.
    int QgcPort = 14550;
    std::string QgcHostIp = "127.0.0.1";

    size_t status_messages_MaxSize = 5000;


    std::shared_ptr<mavlinkcom::MavLinkNode> main_node_;
    std::shared_ptr<mavlinkcom::MavLinkConnection> connection_;
    std::shared_ptr<mavlinkcom::MavLinkVideoServer> video_server_;
    std::shared_ptr<DroneControlBase> drone_control_;

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

    bool is_any_heartbeat_ = false, is_hil_mode_set_ = false, is_armed_ = false;
    bool is_controls_0_1_ = true; //Are motor controls specified in 0..1 or -1..1?
    float rotor_controls_[RotorControlsCount] = {};
    std::queue<std::string> status_messages_;
    int hil_state_freq_ = -1;
    bool actuators_message_supported_ = false;
    const MultiRotor* vehicle_ = nullptr;
    long last_gps_time = 0;
    bool was_reseted = false;


    void normalizeRotorControls()
    {
        if (!is_controls_0_1_) {
            // change -1 to 1 to 0 to 1.
            for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                rotor_controls_[i] = (rotor_controls_[i] + 1.0f) / 2.0f;
            }
        }
    }

    void initializeHILSubscrptions()
    {
        is_any_heartbeat_ = false;
        is_hil_mode_set_ = false;
        is_armed_ = false;
        is_controls_0_1_ = true;
        Utils::setValue(rotor_controls_, 0.0f);
        //TODO: main_node_->setMessageInterval(...);
        auto subscriber = std::bind(&impl::hILSubscriber, this, std::placeholders::_1, std::placeholders::_2);
        connection_->subscribe(subscriber);
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
        catch (std::runtime_error) {
            return false;
        }
    }

    bool connectToLogViewer()
    {
        //set up logviewer proxy
        if (LogViewerHostIp.size() > 0) {
            logviewer_proxy_ = createProxy("LogViewer", LogViewerHostIp, LogViewerPort);
            if (!sendTestMessage(logviewer_proxy_)) {
                // error talking to log viewer, so don't keep trying!
                logviewer_proxy_ = nullptr;
            }
        }
        return logviewer_proxy_ != nullptr;
    }

    bool connectToQGC()
    {
        if (QgcHostIp.size() > 0) {
            qgc_proxy_ = createProxy("QGC", QgcHostIp, QgcPort);
            if (!sendTestMessage(qgc_proxy_)) {
                // error talking to QGC, so don't keep trying!
                qgc_proxy_ = nullptr;
            }
        }
        return qgc_proxy_ != nullptr;
    }


    DroneControlBase* createOrGetDroneControl()
    {
        if (drone_control_ == nullptr) {
            if (connection_ == nullptr)
                throw std::domain_error("MavLinkHelper requires connection object to be set before createOrGetDroneControl call");
            drone_control_ = std::static_pointer_cast<DroneControlBase>(
                std::make_shared<MavLinkDroneControl>(AirControlSysID, AirControlCompID, connection_));
        }

        return drone_control_.get();
    }

    std::shared_ptr<mavlinkcom::MavLinkNode> createProxy(std::string name, std::string ip, int port)
    {
        auto connection = MavLinkConnection::connectRemoteUdp("Proxy to: " + name + " at " + ip + ":" + std::to_string(port), LocalHostIp, ip, port);

        // it is ok to reuse the simulator sysid and compid here because this node is only used to send a few messages directly to this endpoint
        // and all other messages are funnelled through from PX4 via the Join method below.
        auto node = std::make_shared<MavLinkNode>(SimSysID, SimCompID); //TODO: use -1 to autoset sys id on first heartbeat)
        node->connect(connection);

        // now join the main connection to this one, this causes all PX4 messages to be sent to the proxy and all messages from the proxy will be
        // send directly to the PX4 (using whatever sysid/compid comes from that remote node).
        connection_->join(connection);

        return node;
    }

    std::string findPixhawk()
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

    void connectToExternalSim()
    {
        close();

        connection_ = MavLinkConnection::connectLocalUdp("ext_sim", LocalHostIp, ExternalSimPort);
        main_node_ = std::make_shared<MavLinkNode>(ExtRendererSysID, ExtRendererCompID);
        main_node_->connect(connection_);
        main_node_->setMessageInterval(static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_ATT_POS_MOCAP), 50);
        connection_->subscribe(std::bind(&impl::externalSimSubscriber, this, std::placeholders::_1, std::placeholders::_2));
        connectToVideoServer();
    }

    void connectToHIL(const HILConnectionInfo& connection_info)
    {
        createHILConnection(connection_info);
        initializeHILSubscrptions();
    }

    void createHILConnection(const HILConnectionInfo& connection_info)
    {
        if (connection_info.use_serial) {
            createHILSerialConnection(connection_info.serial_port, connection_info.baud_rate);
        }
        else {
            createHILUdpConnection(connection_info.ip_address, connection_info.ip_port);
        }
        connectToVideoServer();
    }

    void createHILUdpConnection(const std::string& ip, int port)
    {
        close();
        connection_ = MavLinkConnection::connectLocalUdp("hil", ip, port);
        main_node_ = std::make_shared<MavLinkNode>(SimSysID, SimCompID); //TODO: use -1 to autoset sys id on first heartbeat
        main_node_->connect(connection_);
    }

    void createHILSerialConnection(const std::string& port_name, int baud_rate)
    {
        close();

        std::string port_name_auto = port_name;
        if (port_name_auto == "" || port_name_auto == "*") {
            port_name_auto = findPixhawk();
        }

        connection_ = MavLinkConnection::connectSerial("hil", port_name_auto, baud_rate);
        main_node_ = std::make_shared<MavLinkNode>(SimSysID, SimCompID); //TODO: use -1 to autoset sys id on first heartbeat
        main_node_->connect(connection_);
    }

    void connectToVideoServer()
    {
        video_server_ = std::make_shared<MavLinkVideoServer>(ExtRendererSysID, ExtRendererCompID);
        video_server_->connect(connection_);
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

    void getStatusMessages(std::vector<std::string>& messages)
    {
        messages.clear();
        std::lock_guard<std::mutex> guard(status_text_mutex_);

        while (!status_messages_.empty()) {
            messages.push_back(status_messages_.front());
            status_messages_.pop();
        }
    }

    void hILSubscriber(std::shared_ptr<MavLinkConnection> connection, const MavLinkMessage& msg)
    {
        processHILMessages(msg);
    }

    void processHILMessages(const MavLinkMessage& msg)
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
            } else if (!is_hil_mode_set_) {
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
        hil_sensor.fields_updated = was_reseted ? (1 << 31) : 0;

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

    real_T getRotorControlSignal(unsigned int rotor_index)
    {
        std::lock_guard<std::mutex> guard(hil_controls_mutex_);
        return rotor_controls_[rotor_index];
    }

    void initialize(const MultiRotor* vehicle)
    {
        vehicle_ = vehicle;
    }

    //*** Start: UpdatableState implementation ***//
    void reset()
    {
        was_reseted = true;
        Utils::setValue(rotor_controls_, 0.0f);
        last_gps_time = 0;
        setNormalMode();
    }

    void update(real_T dt)
    {
        if (connection_ == nullptr || !connection_->isOpen())
            return;

        //send sensor updates
        const auto& imu_output = vehicle_->getImu()->getOutput();
        const auto& mag_output = vehicle_->getMagnetometer()->getOutput();
        const auto& baro_output = vehicle_->getBarometer()->getOutput();

        sendHILSensor(imu_output.linear_acceleration, 
            imu_output.angular_velocity, 
            mag_output.magnetic_field_body,
            baro_output.pressure * 0.01f /*Pa to Milibar */, baro_output.altitude);

        if (vehicle_->getGps() != nullptr) {
            const auto& gps_output = vehicle_->getGps()->getOutput();

            //send GPS
            if (gps_output.is_valid && gps_output.gnss.time_utc > last_gps_time) {
                last_gps_time = gps_output.gnss.time_utc;
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
        if (was_reseted)
            was_reseted = false;
    }
    //*** End: UpdatableState implementation ***//

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
            SetModeMessage.target_system = SimSysID;
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
        SetModeMessage.target_system = SimSysID;
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
    }

};

//empty constructor required for pimpl
MavLinkHelper::MavLinkHelper()
{
    pimpl_.reset(new impl());
}

MavLinkHelper::~MavLinkHelper()
{
    pimpl_->close();
}

int MavLinkHelper::getRotorControlsCount()
{
    return RotorControlsCount;
}
void MavLinkHelper::connectToExternalSim()
{
    pimpl_->connectToExternalSim();
}
void MavLinkHelper::connectToHIL(const HILConnectionInfo& connection_info)
{
    pimpl_->connectToHIL(connection_info);
}
bool MavLinkHelper::connectToLogViewer()
{
    return pimpl_->connectToLogViewer();
}
bool MavLinkHelper::connectToQGC()
{
    return pimpl_->connectToQGC();
}
void MavLinkHelper::connectToVideoServer()
{
    pimpl_->connectToVideoServer();
}
void MavLinkHelper::sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
{
    pimpl_->sendImage(data, length, width, height);
}
void MavLinkHelper::getMocapPose(Vector3r& position, Quaternionr& orientation)
{
    pimpl_->getMocapPose(position, orientation);
}
void MavLinkHelper::sendMocapPose(const Vector3r& position, const Quaternionr& orientation)
{
    pimpl_->sendMocapPose(position, orientation);
}
void MavLinkHelper::sendCollison(float normalX, float normalY, float normalZ)
{
    pimpl_->sendCollison(normalX, normalY, normalZ);
}
bool MavLinkHelper::hasVideoRequest()
{
    return pimpl_->hasVideoRequest();
}
void MavLinkHelper::sendHILSensor(const Vector3r& acceleration, const Vector3r& gyro, const Vector3r& mag, float abs_pressure, float pressure_alt)
{
    pimpl_->sendHILSensor(acceleration, gyro, mag, abs_pressure, pressure_alt);
}
void MavLinkHelper::sendHILGps(const GeoPoint& geo_point, const Vector3r& velocity, float velocity_xy, float cog, float eph, float epv, int fix_type, unsigned int satellites_visible)
{
    pimpl_->sendHILGps(geo_point, velocity, velocity_xy, cog, eph, epv, fix_type, satellites_visible);
}
void MavLinkHelper::getStatusMessages(std::vector<std::string>& messages)
{
    pimpl_->getStatusMessages(messages);
}
void MavLinkHelper::close()
{
    pimpl_->close();
}
void MavLinkHelper::setNormalMode()
{
    pimpl_->setNormalMode();
}
void MavLinkHelper::setHILMode()
{
    pimpl_->setHILMode();
}
std::string MavLinkHelper::findPixhawk()
{
    return pimpl_->findPixhawk();
}
void MavLinkHelper::initialize(const MultiRotor* vehicle)
{
    pimpl_->initialize(vehicle);
}
real_T MavLinkHelper::getRotorControlSignal(unsigned int rotor_index)
{
    return pimpl_->getRotorControlSignal(rotor_index);
}
DroneControlBase* MavLinkHelper::createOrGetDroneControl()
{
    return pimpl_->createOrGetDroneControl();
}


//*** Start: UpdatableState implementation ***//
void MavLinkHelper::reset()
{
    pimpl_->reset();
}
void MavLinkHelper::update(real_T dt)
{
    pimpl_->update(dt);
}
//*** End: UpdatableState implementation ***//

}} //namespace
#endif

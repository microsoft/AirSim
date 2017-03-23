// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Utils.hpp"
#include "Commands.h"
#include <chrono>
#include <math.h>
#include <iostream>
#include <string.h>
#include <thread>
#include <string>
#include "MavLinkMessages.hpp"
#include "FileSystem.hpp"

using namespace mavlink_utils;
using namespace mavlinkcom;

// from main.cpp.

static std::vector<Command*> const * all_commands_;
std::vector<Command*> const * Command::getAllCommand() { return all_commands_;  }
void Command::setAllCommand(std::vector<Command*> const * all_commands) { all_commands_ = all_commands; }

void mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw);
void PrintHeartbeat(const MavLinkMessage& msg);

Command::Command()
{
}


Command::~Command()
{
	Close();
}

// you must call this method if you want HandleMessage to be called subsequently.
void Command::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	if (this->vehicle != com) {
		Close();
	}
	this->vehicle = com;
	
    if (subscription == 0)
    {
        subscription = com->getConnection()->subscribe([=](std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg) {
            try {
                HandleMessage(msg);
            }
            catch (std::exception& e) {
                Utils::logError("### Runtime Error: %s", e.what());
            }
            catch (...) {
                Utils::logError("### unhandled exception");
            }
        });
    }
}

void Command::Close() {

	if (subscription != 0 && vehicle != nullptr) {
		vehicle->getConnection()->unsubscribe(subscription);
		subscription = 0;
	}
	vehicle = nullptr;
}


std::vector<std::string> Command::parseArgs(std::string s)
{
    auto start = s.begin();
    std::vector<std::string> result;
    auto theEnd = s.end();
    auto it = s.begin();
    while (it != theEnd)
    {
        char ch = *it;
        if (ch == ' ' || ch == '\t' || ch == ',') {
            if (start < it)
            {
                result.push_back(std::string(start, it));
            }
            it++;
            start = it;
        }
        else if (*it == '"')
        {
            // treat literals as one word
            it++;
            start = it;
            while (*it != '"' && it != theEnd)
            {
                it++;
            }
            auto end = it;
            if (start < it)
            {
                result.push_back(std::string(start, end));
            }
            if (*it == '"') {
                it++;
            }
            start = it;
        }
        else {
            it++;
        }
    }
    if (start < theEnd)
    {
        result.push_back(std::string(start, s.end()));
    }
    return result;
}

Command* Command::create(const std::vector<std::string>& args)
{
    if (all_commands_ == nullptr)
        throw std::runtime_error("all_commands_ member must be set before calling Command::parseLine()");

    const auto& cmdTable = *all_commands_;

    Command* selected = nullptr;
    for (size_t i = 0; i < cmdTable.size(); i++)
    {
        Command* command = cmdTable.at(i);
        if (command->Parse(args))
            return command;
    }

    return nullptr;
}

Command* Command::create(const std::string& line)
{
    std::vector<std::string> args = Command::parseArgs(line);
    return create(args);
}

bool ArmDisarmCommand::Parse(const std::vector<std::string>& args) {
	this->arm = false;
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "arm")
		{
			this->arm = true;
			return true;
		}
		else if (cmd == "disarm") {
			this->arm = false;
			return true;
		}
	}
	return false;
}




void ArmDisarmCommand::Execute(std::shared_ptr<MavLinkVehicle> com) {

	bool rc = false;
	if (com->allowFlightControlOverUsb().wait(3000, &rc)) {
		if (!rc) {
			printf("failed to set CBRK_USB_CHK\n");
		}
	}
	else {
		printf("timeout waiting for set CBRK_USB_CHK\n");
	}

	if (com->armDisarm(arm).wait(3000, &rc)) {
		if (rc) {
			printf("ok\n");
		}
		else {
			printf("failed\n");
		}
	}
	else {
		printf("timeout waiting for armDisarm ACK\n");
	}
}

bool GetParamsCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "params") {

			if (args.size() > 1)
			{
				OpenLog(args[1].c_str());
			}
			return true;
		}
	}
	return false;
}
void GetParamsCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	try {
		auto list = com->getParamList();
		auto end = list.end();
		int count = 0;
		for (auto iter = list.begin(); iter < end; iter++)
		{
			count++;
			MavLinkParameter p = *iter;
			if (p.type == static_cast<int>(MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32) || 
                p.type == static_cast<int>(MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL64)) {
				if (ptr != nullptr) {
					fprintf(ptr, "%s=%f\n", p.name.c_str(), p.value);
				}
				else {
					printf("%s=%f\n", p.name.c_str(), p.value);
				}
			}
			else {
				if (ptr != nullptr) {
					fprintf(ptr, "%s=%d\n", p.name.c_str(), static_cast<int>(p.value));
				}
				else {
					printf("%s=%d\n", p.name.c_str(), static_cast<int>(p.value));
				}
			}
		}
		printf("found %d parameters\n", count);
	}
	catch (const std::exception& e) {
		printf("%s\n", e.what());
	}
	Close();
}

bool GetSetParamCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "param") {
			get = false;
			value = 0;
			if (args.size() > 1)
			{
				cmd = args[1];
				if (cmd == "set") {
					if (args.size() > 2)
					{
						name = args[2];
						if (args.size() > 3)
						{
							value = static_cast<float>(atof(args[3].c_str()));
						}
						else {
							printf("Missing 'value' argument on param set command\n");
							return false;
						}
					}
					else {
						printf("Missing 'name' argument on param set command\n");
						return false;
					}
				}
				else if (cmd == "get")
				{
					get = true;
					if (args.size() > 2)
					{
						name = args[2];
					}
					else {
						printf("Missing 'name' argument on param get command\n");
						return false;
					}
				}
			}
			return true;
		}
	}
	return false;
}

void GetSetParamCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	if (get) {
		try {
			com->getParameter(name).then([=](MavLinkParameter  p) {
				if (p.type == static_cast<int>(MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL32) || 
                    p.type == static_cast<int>(MAV_PARAM_TYPE::MAV_PARAM_TYPE_REAL64)) {
					printf("%s=%f\n", p.name.c_str(), p.value);
				}
				else {
					printf("%s=%d\n", p.name.c_str(), static_cast<int>(p.value));
				}
			});
		}
		catch (const std::exception& e)
		{
			printf("Exception: %s\n", e.what());
		}
	}
	else {
		try {
			MavLinkParameter  p;
			p.name = name;
			p.index = -1;
			p.value = value;
			printf("setting param %s to new value %f\n", name.c_str(), value);
			com->setParameter(p);
			printf("success\n");
		}
		catch (const std::exception& e)
		{
			printf("Exception: %s\n", e.what());
		}
	}
}

bool TakeOffCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "takeoff") {
            altitude = 5; // default
			if (args.size() > 1)
			{
				altitude = static_cast<float>(atof(args[1].c_str()));
			}
			return true;
		}
	}
	return false;
}

void TakeOffCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	// subscribe to get subsequent messages so we can track our progress towards the requested altitude.
	Command::Execute(com);

	// request gps info
	reached = false;
	offground = false;
	bool rc = false;
	if (com->takeoff(-altitude).wait(3000, &rc)) {
		if (rc) {
			printf("ok\n");
		}
		else {
			printf("failed\n");
		}
	} else {
		printf("Timeout waiting for ACK from takeoff command\n");
	}
}


bool PlayLogCommand::Parse(const std::vector<std::string>& args)
{
    if (args.size() <= 0)
        return false;
    
    std::string cmd = args[0];
    if (cmd == "playlog") {
        if (args.size() > 1) {
            log_.openForReading(args.at(1));
            return true;
        } 
        else {
            printf("Usage: playlog <mavlink_logfile>\n");
        }
    }
    return false;
}

void PlayLogCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
    MavLinkMessage msg;
    std::fill_n(quaternion_, 4, 0.0f);
    quaternion_[0] = 1; //unit quaternion
    uint64_t log_timestamp, log_start_timestamp = 0;
    uint64_t playback_timestamp, playback_start_timestamp;

    playback_timestamp = playback_start_timestamp = MavLinkLog::getTimeStamp();
    int last_basemode = -1, last_custommode = -1;

    while (log_.read(msg, log_timestamp)) {
        if (log_start_timestamp == 0)
            log_start_timestamp = log_timestamp;

        switch (msg.msgid)
        {
        case MavLinkStatustext::kMessageId: {
            //sync clocks
            auto current_timestamp = MavLinkLog::getTimeStamp();
            long waitMicros = static_cast<long>(log_timestamp - log_start_timestamp) - static_cast<long>(current_timestamp - playback_start_timestamp);
            if (waitMicros > 0) {
                if (waitMicros > 1E6) { //1s
                    printf("synchronizing clocks for %f sec\n", waitMicros / 1E6f);
                }
                std::this_thread::sleep_for(std::chrono::microseconds(waitMicros));
            }

            MavLinkStatustext status_msg;
            status_msg.decode(msg);
            if (std::strstr(status_msg.text, kCommandLogPrefix) == status_msg.text) {
                std::string line = std::string(status_msg.text).substr(std::strlen(kCommandLogPrefix));
                auto command = Command::create(line);
                if (command == nullptr)
                    throw std::runtime_error(std::string("Command for line ") + line + " cannot be found");
                printf("Executing %s\n", line.c_str());
                command->Execute(com);
            }
            break;
        }
        case MavLinkLocalPositionNed::kMessageId: {
            MavLinkLocalPositionNed pos_msg;
            pos_msg.decode(msg);
            MavLinkAttPosMocap mocap;
            mocap.x = x = pos_msg.x;
            mocap.y = y = pos_msg.y;
            mocap.z = z = pos_msg.z;
            std::copy(quaternion_, quaternion_ + 4, mocap.q);
            com->writeMessage(mocap);
            break;
        }
        case MavLinkAttitudeQuaternion::kMessageId: {
            MavLinkAttitudeQuaternion q_msg;
            q_msg.decode(msg);
            MavLinkAttPosMocap mocap;
            mocap.x = x;
            mocap.y = y;
            mocap.z = z;

            quaternion_[0] = q_msg.q1; //w
            quaternion_[1] = q_msg.q2; //x
            quaternion_[2] = q_msg.q3; //y
            quaternion_[3] = q_msg.q4; //z

            Utils::copy(quaternion_, mocap.q);
            com->writeMessage(mocap);
            break;
        }
        case MavLinkHeartbeat::kMessageId: {
            MavLinkHeartbeat heartbeat_msg;
            heartbeat_msg.decode(msg);

            if (heartbeat_msg.base_mode != last_basemode || heartbeat_msg.custom_mode != last_custommode) {
                last_basemode = heartbeat_msg.base_mode;
                last_custommode = heartbeat_msg.custom_mode;
                //TODO: avoid passing hadcoded HIL flag
                com->setMode(last_basemode | static_cast<int>(MAV_MODE_FLAG::MAV_MODE_FLAG_HIL_ENABLED), last_custommode);
            }

            break;
        }
        default:
            break;
        }
    }
}

void TakeOffCommand::HandleMessage(const MavLinkMessage& msg)
{
	if (msg.msgid == static_cast<uint8_t>(MavLinkMessageIds::MAVLINK_MSG_ID_GPS_RAW_INT))
	{
		// The global position, as returned by the Global Positioning System (GPS).		
		MavLinkGpsRawInt rawGps;
		rawGps.decode(msg);
		float lat = static_cast<float>(static_cast<double>(rawGps.lat) / 1e7);
		float lon = static_cast<float>(static_cast<double>(rawGps.lon) / 1e7);
		float alt = static_cast<float>(static_cast<double>(rawGps.alt) / 1000);
		if (!reached && alt >= targetAlt - delta && alt <= targetAlt + delta)
		{
			reached = true;
			Close(); // stop listening.
			printf("Target altitude reached at lat=%f, long=%f, alt=%f\n", lat, lon, alt);
		}
	}
	else if (msg.msgid == static_cast<uint8_t>(MavLinkMessageIds::MAVLINK_MSG_ID_EXTENDED_SYS_STATE))
	{
		// Provides state for additional features
		// The general system state
		MavLinkExtendedSysState status;
		status.decode(msg);
		if (!offground && status.landed_state == static_cast<uint8_t>(MAV_LANDED_STATE::MAV_LANDED_STATE_IN_AIR))
		{
			printf("Drone has left the ground\n");
			offground = true;
		}
	}
}

bool LandCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "land") {
			return true;
		}
	}
	return false;
}

void LandCommand::Execute(std::shared_ptr<MavLinkVehicle> com) {

	// subscribe to get subsequent messages so we can track our progress towards the requested altitude.
	Command::Execute(com);

	printf("landing drone...\n");
	landed = false;
	const VehicleState& state = com->getVehicleState();
	bool rc = false;
	if (com->land(state.global_est.heading, state.home.global_pos.lat, state.home.global_pos.lon, state.home.global_pos.alt).wait(3000, &rc)) {
		if (rc) {
			printf("ok\n");
		}
		else {
			printf("failed\n");
		}
	} else {
		printf("timeout waiting for ACT from land command\n");
	}
}

void LandCommand::HandleMessage(const MavLinkMessage& message)
{
	if (message.msgid == MavLinkExtendedSysState::kMessageId)
	{
		// Provides state for additional features
		// The general system state
		MavLinkExtendedSysState status;
		status.decode(message);
		if (!landed && status.landed_state == static_cast<uint8_t>(MAV_LANDED_STATE::MAV_LANDED_STATE_ON_GROUND))
		{
			printf("Drone has landed\n");
			Close();
			landed = true;
		}
	}
}

bool RtlCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "rtl") {
			return true;
		}
	}
	return false;
}
void RtlCommand::Execute(std::shared_ptr<MavLinkVehicle> com) {

	// subscribe to get subsequent messages so we can track our progress towards the requested altitude.
	Command::Execute(com);

	printf("Returning back to launch point...\n");
	bool rc = false;
	if (com->returnToHome().wait(5000, &rc)) {
		if (rc) {
			printf("ok\n");
		}
		else {
			printf("failed\n");
		}
	}
	else {
		printf("timeout waiting for ACK for RTL command\n");
	}
}

void RtlCommand::HandleMessage(const MavLinkMessage& message)
{
	if (message.msgid == MavLinkExtendedSysState::kMessageId)
	{
		// Provides state for additional features
		// The general system state
		MavLinkExtendedSysState status;
		status.decode(message);
		if (!landed && status.landed_state == static_cast<uint8_t>(MAV_LANDED_STATE::MAV_LANDED_STATE_ON_GROUND))
		{
			printf("Drone has landed\n");
			Close();
			landed = true;
		}
	}
}

bool LoiterCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "loiter") {
			return true;
		}
	}
	return false;
}
void LoiterCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	bool rc = false;
	if (com->loiter().wait(2000, &rc)) {
		if (rc) {
			printf("ok\n");
		}
		else {
			printf("failed\n");
		}
	}
	else {
		printf("timeout waiting for ACT from loiter command\n");
	}
}

bool RequestImageCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "req_img") {
			return true;
		}
	}
	return false;
}

void RequestImageCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	Command::Execute(com);
	stream = std::make_shared<MavLinkVideoClient>(150, 1);
	stream->connect(com->getConnection());
	stream->requestVideo(0, 1E7, false);
	printf("requested\n");
}

void RequestImageCommand::HandleMessage(const MavLinkMessage& msg)
{
	if (msg.msgid == static_cast<uint8_t>(MavLinkMessageIds::MAVLINK_MSG_ID_ENCAPSULATED_DATA))
	{
		MavLinkVideoClient::MavLinkVideoFrame image;
		if (stream->readNextFrame(image))
		{
			printf("image received of size %d\n", static_cast<int>(image.data.size()));
			Close();
		}
	}
}

bool MissionCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "mission") {
			return true;
		}
	}
	return false;
}
void MissionCommand::Execute(std::shared_ptr<MavLinkVehicle> com) {

	Command::Execute(com);
	printf("Executing preprogrammed mission (if there is one)...\n");
	com->setMissionMode();
}

void MissionCommand::HandleMessage(const MavLinkMessage& message)
{
	if (message.msgid == MavLinkExtendedSysState::kMessageId)
	{
		// Provides state for additional features
		// The general system state
		MavLinkExtendedSysState status;
		status.decode(message);
		if (!landed && status.landed_state == static_cast<uint8_t>(MAV_LANDED_STATE::MAV_LANDED_STATE_ON_GROUND))
		{
			printf("Drone has landed\n");
			Close();
			landed = true;
		}
	}
}

bool PositionCommand::Parse(const std::vector<std::string>& args) {

	this->printLocalPosition = true;
	this->printGlobalosition = true;
	this->setHome = false;

	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "pos" || cmd == "position") {
			if (args.size() > 1)
			{
				std::string arg = args[1];
				if (arg == "home") {
					this->setHome = true;
				}
			}
			return true;
		}
	}
	return false;
}
void PositionCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	Command::Execute(com);
}

void PositionCommand::HandleMessage(const MavLinkMessage& message)
{
	if (printGlobalosition && message.msgid == MavLinkGlobalPositionInt::kMessageId) {
		printGlobalosition = false;
		MavLinkGlobalPositionInt pos;
		pos.decode(message);
		float lat = static_cast<float>(pos.lat) / 1E7f;
		float lon = static_cast<float>(pos.lon) / 1E7f;
		float alt = static_cast<float>(pos.alt) / 1E3f;
		float alt_ground = static_cast<float>(pos.relative_alt) / 1000;
		float heading = static_cast<float>(pos.hdg) / 100;
		printf("Global Position: lat=%f, lon=%f, alt=%f, relalt=%f, heading=%f\n", lat, lon, alt, alt_ground, heading);

		if (this->setHome) {
			this->setHome = false;
			if (!this->vehicle->getVehicleState().controls.landed) {
				printf("Cannot set home position because it is not in the 'landed' state\n");
			}
			else {
				this->vehicle->setHomePosition(lat, lon, alt);
				printf("Setting Home Position: lat=%f, lon=%f, alt=%f\n", lat, lon, alt);
			}
		}
	}
	if (printLocalPosition && message.msgid == MavLinkLocalPositionNed::kMessageId) {
		printLocalPosition = false;
		MavLinkLocalPositionNed pos;
		pos.decode(message);
		printf("Local Position: x=%f, y=%f, z=%f\n", pos.x, pos.y, pos.z);

	}
}

bool BatteryCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "battery") {
			return true;
		}
	}
	return false;
}

void BatteryCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	Command::Execute(com);
	got_battery_ = false;
}

void BatteryCommand::HandleMessage(const MavLinkMessage& message)
{
	if (!got_battery_ && MavLinkBatteryStatus::kMessageId)
	{
		MavLinkBatteryStatus status;
		status.decode(message);
		got_battery_ = true;
		printf("Battery voltage %d mV\n", status.current_battery);
	}
}


bool StatusCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "status") {
			return true;
		}
	}
	return false;
}
void StatusCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	Command::Execute(com);
	printStatus = true;
	printExtStatus = true;
	printHeartbeat = true;
	printHomePosition = true;
}

extern void PrintSystemStatus(MavLinkSysStatus& status);


// Enumeration of landed detector states
const static char* MAV_LANDED_STATE_NAMES[]{
	// MAV landed state is unknown
	"MAV_LANDED_STATE_UNDEFINED",
	// MAV is landed (on ground)
	"MAV_LANDED_STATE_ON_GROUND",
	// MAV is in air
	"MAV_LANDED_STATE_IN_AIR",
};


void StatusCommand::HandleMessage(const MavLinkMessage& message)
{
	if (printStatus && MavLinkSysStatus::kMessageId)
	{
		MavLinkSysStatus status;
		status.decode(message);

		PrintSystemStatus(status);
		printStatus = false;
	}
	if (printExtStatus && message.msgid == MavLinkExtendedSysState::kMessageId)
	{
		MavLinkExtendedSysState status;
		status.decode(message);

		int ls = static_cast<int>(status.landed_state);
		if (ls < static_cast<int>(sizeof(MAV_LANDED_STATE_NAMES))) {
			printf("    landed_state=%s\n", MAV_LANDED_STATE_NAMES[ls]);
		}
		printf("    vtol_state=%d\n", static_cast<int>(status.vtol_state));
		printExtStatus = false;
	}
	if (printHeartbeat && message.msgid == MavLinkHeartbeat::kMessageId) {
		printHeartbeat = false;
		PrintHeartbeat(message);
	}

	if (printHomePosition && message.msgid == MavLinkHomePosition::kMessageId) {
		printHomePosition = false;
		MavLinkHomePosition home;
		home.decode(message);
		float lat = static_cast<float>(home.latitude) / 1E7f;
		float lon = static_cast<float>(home.longitude) / 1E7f;
		float alt = static_cast<float>(home.altitude) / 1E3f;
		printf("Home Position: lat=%f, lon=%f, alt=%f\n", lat, lon, alt);
	}

}

bool SendImageCommand::Parse(const std::vector<std::string>& args) {
	fileName = "";
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "sendimage") {
			if (args.size() == 4)
			{
				fileName = Utils::trim(args[1], '"');
				width = atoi(args[2].c_str());
				height = atoi(args[3].c_str());
				return true;
			}
			else
			{
				printf("sendimage is missing some argument\n");
			}
		}
	}
	return false;
}

class StreamBuffer {
	size_t size = 0;

public:
	size_t length = 0;
	char* buffer = nullptr;

	void Write(char* bytes, size_t len)
	{
		if (length + len > size)
		{
			size_t newLen = size * 2;
			if (newLen < size + len) {
				newLen = size + len;
			}
			char* newBuffer = new char[newLen];
			if (newBuffer == nullptr) {
				throw std::runtime_error("out of memory");
			}
			if (size > 0) {
				::memcpy(newBuffer, buffer, size);
			}
			buffer = newBuffer;
			size = newLen;
		}
		::memcpy(&buffer[length], bytes, len);
		length += len;
	}

	~StreamBuffer() {
		if (buffer != nullptr) {
			delete[] buffer;
		}
	}
};

void SendImageCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	if (logViewer.get() == nullptr)
	{
		printf("sendimage needs a logviewer (use -log on the command line)\n");
		return;
	}
	std::string ext = Utils::getFileExtension(fileName);
	ext = Utils::toLower(ext);
	uint8_t type = 0;
	if (ext == ".png") {
		type = static_cast<uint8_t>(MAVLINK_DATA_STREAM_TYPE::MAVLINK_DATA_STREAM_IMG_PNG);
	}
	else if (ext == ".jpg") {
		type = static_cast<uint8_t>(MAVLINK_DATA_STREAM_TYPE::MAVLINK_DATA_STREAM_IMG_JPEG);
	}
	else if (ext == ".bmp") {
		type = static_cast<uint8_t>(MAVLINK_DATA_STREAM_TYPE::MAVLINK_DATA_STREAM_IMG_BMP);
	}
	else {
		printf("unsupported image type: %s\n", ext.c_str());
		return;
	}

	FILE* fptr = fopen(fileName.c_str(), "rb");
	if (fptr == nullptr)
	{
		printf("Error opening file '%s'\n", fileName.c_str());
		return;
	}

	size_t BUFSIZE = 65536;
	StreamBuffer sbuf;
	char* temp = new char[BUFSIZE];
	size_t len = 0;
	while (true) {

		size_t i = fread(temp, 1, BUFSIZE, fptr);
		if (i == 0) {
			break;
		}
		else {
			sbuf.Write(temp, i);
			len += i;
		}
	}
	delete[] temp;
	fclose(fptr);

	MavLinkVideoServer stream{ 1, 1 };
	stream.connect(logViewer->getConnection());
	stream.sendFrame(reinterpret_cast<uint8_t*>(sbuf.buffer), static_cast<uint32_t>(len), width, height, type, 100);
}

bool CapabilitiesCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "cap" || cmd == "capabilities") {
			return true;
		}
	}
	return false;
}
void CapabilitiesCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	MavLinkAutopilotVersion ver;
	if (com->getCapabilities().wait(10000, &ver)) 
	{
		printf("AUTOPILOT_VERSION: \n");
		printf("    capabilities: %lx\n", static_cast<long unsigned int>(ver.capabilities));

		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_MISSION_INT)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_MISSION_INT\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_PARAM_UNION)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_PARAM_UNION\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_FTP)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_FTP\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_TERRAIN)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_TERRAIN\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION\n");
		}
		if ((ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION)) != 0) {
			printf("      MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION\n");
		}

		printf("    flight_sw_version: %d\n", ver.flight_sw_version);
		printf("    middleware_sw_version: %d\n", ver.middleware_sw_version);
		printf("    os_sw_version: %d\n", ver.os_sw_version);
		printf("    board_version: %d\n", ver.board_version);
		printf("    vendor_id: %d\n", static_cast<int>(ver.vendor_id));
		printf("    product_id: %d\n", static_cast<int>(ver.product_id));
		printf("    product_id: %ld\n", static_cast<long>(ver.uid));
		Close();
	}
	else {
		printf("### capabilities are not being returned after 10 seconds");
	}
}

bool GotoCommand::Parse(const std::vector<std::string>& args) {
	cruise_speed_ = 0;
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "goto") {
			if (args.size() >= 4) {
				tx = static_cast<float>(atof(args[1].c_str()));
				ty = static_cast<float>(atof(args[2].c_str()));
				tz = static_cast<float>(atof(args[3].c_str()));
                if (tz > 0) {
                    tz = -tz; // probably mean negative, since we are using NED coordinates.
                }
				if (fabs(tx) > 100) {
					printf("### invalid -100 < x < 100 \n");
					return false;
				}
				if (fabs(ty) > 100) {
					printf("### invalid -100 < y < 100 \n");
					return false;
				}
				if (fabs(tz) > 100) {
					printf("### invalid -100 < z < 100 \n");
					return false;
				}

				if (args.size() == 5) {
					cruise_speed_ = static_cast<float>(atof(args[4].c_str()));		
					if (cruise_speed_ < 0 || cruise_speed_ > 10) {
						printf("### invalid speed, 0 < speed < 10\n");
						return false;
					}
				}
				return true;
			}
			else {
				printf("goto - expecting x,y,z arguments\n");
			}
		}
	}
	return false;
}

void GotoCommand::Execute(std::shared_ptr<MavLinkVehicle> com) {
	this->channel = com;
	this->requestedControl = false;
	this->hasControl = false;
	this->targetReached = false;
	this->settled = false;
	this->hasLocalPosition = false;
	this->targetPosition = false;
	this->targetVelocity = false;
	this->targetVelocityAltHold = false;
	this->targetSpeed = 0;
	this->theading = 0;
	this->paused = false;
	Command::Execute(com);

	TakeControl();
}

void GotoCommand::TakeControl()
{
	if (!this->requestedControl) {
		// control works better if we get about 50 of these per second (20ms interval, if we can).
		this->channel->requestControl();
		this->requestedControl = true;
	}
}

void GotoCommand::Close()
{
	if (this->requestedControl && vehicle != nullptr && vehicle->hasOffboardControl()) {
		vehicle->releaseControl();
	}
	this->requestedControl = false;
	Command::Close();
}

void GotoCommand::HandleMessage(const MavLinkMessage& message)
{
	switch (message.msgid)
	{
	case MavLinkLocalPositionNed::kMessageId: // MAVLINK_MSG_ID_LOCAL_POSITION_NED:
	{
		// The filtered local position
		MavLinkLocalPositionNed localPos;
		localPos.decode(message);

		this->vx = localPos.vx;
		this->vy = localPos.vy;
		this->vz = localPos.vz;
		this->x = localPos.x;
		this->y = localPos.y;
		this->z = localPos.z;

		if (paused) {
			return;
		}

		if (!this->requestedControl || ! channel->hasOffboardControl()) {
			return;
		}
		
		if (!this->hasLocalPosition) {
			this->hasLocalPosition = true;
			HasLocalPosition();
		}

		UpdateTarget();

		if (targetPosition) {
			// must send these regularly to keep offboard control.
			channel->moveToLocalPosition(tx, ty, tz, is_yaw, static_cast<float>(theading * M_PI / 180));
			
			if (this->hasLocalPosition) {
				if (!targetReached && fabsf(x - tx) < nearDelta && fabsf(y - ty) < nearDelta)
				{
					targetReached = true;
				}
				if (targetReached && !settled && (fabs(this->vx) + fabsf(this->vy) + fabsf(this->vz) < almostStationery)) {
					settled = true;
					// ok, now we can safely switch to loiter.
					TargetReached();
				}
			}
		}
		else if (targetVelocity) {

			// must send these regularly to keep offboard control.
			// integrate the heading so it is smoother.
			channel->moveByLocalVelocity(tvx, tvy, tvz, is_yaw, static_cast<float>(theading * M_PI / 180));
		}
		else if (targetVelocityAltHold) {

			// must send these regularly to keep offboard control.
			// integrate the heading so it is smoother.
			channel->moveByLocalVelocityWithAltHold(tvx, tvy, tz, is_yaw, static_cast<float>(theading * M_PI / 180));
		}
		break;
	}
	case MavLinkAttitude::kMessageId: // MAVLINK_MSG_ID_ATTITUDE:
	{
		MavLinkAttitude att;
		att.decode(message);
		pitch = att.pitch;
		pitchSpeed = att.pitchspeed;
		roll = att.roll;
		rollSpeed = att.rollspeed;
		yaw = att.yaw;
		yawSpeed = att.yawspeed;
		break;
	}
	default:
		break;
	}
}

void GotoCommand::UpdateTarget()
{

}

void GotoCommand::HasLocalPosition() {
	Goto(tx, ty, tz, -1, yaw);
}

void GotoCommand::Goto(float targetX, float targetY, float targetZ, float speed, float heading, bool isYaw)
{
	targetPosition = true;
	targetVelocity = false;
	targetVelocityAltHold = false;
	tx = targetX;
	ty = targetY;
	tz = targetZ;
	is_yaw = isYaw;
	theading = heading;
	targetSpeed = speed;
	targetReached = false;
}

void GotoCommand::Move(float targetvx, float targetvy, float targetvz, float heading, bool isYaw)
{
	targetPosition = false;
	targetVelocity = true;
	targetVelocityAltHold = false;
	tvx = targetvx;
	tvy = targetvy;
	tvz = targetvz;
	is_yaw = isYaw;
	theading = heading;
}

void GotoCommand::MoveAltHold(float targetvx, float targetvy, float targetZ, float heading, bool isYaw)
{
	targetPosition = false;
	targetVelocity = false;
	targetVelocityAltHold = true;
	tvx = targetvx;
	tvy = targetvy;
	tz = targetZ;
	is_yaw = isYaw;
	theading = heading;
}

void GotoCommand::TargetReached()
{
	targetReached = true;

	printf("target reached\n");
}

bool OrbitCommand::Parse(const std::vector<std::string>& args) {
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "orbit") {
            radius = 0;
			if (args.size() > 1) {
				radius = static_cast<float>(atof(args[1].c_str()));
                if (radius < 1 || radius > 100) {
                    printf("radius '%f' is invalid, expecting 1 < radius < 100\n", radius);
                    return false;
                }
			}
			else
			{
				printf("orbit - expecting radius argument\n");
				return false;
			}
			speed = 1;
			if (args.size() > 2) {
				speed = static_cast<float>(atof(args[2].c_str()));
                if (speed < 0.1 || speed > 10) {
                    printf("speed '%f' is invalid, expecting 0.1 < radius < 10\n", speed);
                    return false;
                }
			}
			return true;
		}
	}
	return false;
}

void OrbitCommand::Execute(std::shared_ptr<MavLinkVehicle> com) {

	if (!com->isLocalControlSupported()) {
		throw std::runtime_error(Utils::stringf("Your drone does not support the MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED capability."));
	}

	printf("orbit current position at radius %f\n", static_cast<float>(radius));
	orbiting = false;
	flyingToRadius = false;
	startAngle = 0;
	startTime = 0;
	orbits = 0;
	halfWay = false;    
	previousAngle = 0;	
	orbitSpeed = 0;
	printf("waiting for local position...\n");
    GotoCommand::Execute(com);
}

void OrbitCommand::HasLocalPosition() {

	// fly to radius
	flyingToRadius = true;
	cx = x;
	cy = y;
	cz = z;

	orbiting = false;
	tx = x + radius;
	ty = y;
	tz = cx;
	printf("Flying to orbit rim...\n");
}

double pin(double value, double max) {
	if (value > max) {
		return max;
	}
	if (value < -max) {
		return -max;
	}
	return value;
}

void OrbitCommand::UpdateTarget()
{
	const float noMovement = 0.5;

	if (flyingToRadius) {

		double newvx = 1;
		double newvy = ty - y;
		newvy = pin(newvy, 1);
		// it takes about 10 cm to stop and turn.
		if (x + 0.1 > tx) {
			// next time around switch to orbiting!
			printf("Reached the rim of our orbit, switching to circular flight...\n");
			orbiting = true;
			orbitSpeed = 0;
			flyingToRadius = false;
		}		
		MoveAltHold(static_cast<float>(newvx), static_cast<float>(newvy), cz, 180);
	}
	else if (orbiting)
	{
		// heading points to center of circle.
		float dx = x - cx;
		float dy = y - cy;
		float angle = atan2(dy, dx);

		if (angle < 0) {
			angle += static_cast<float>(M_PI * 2);
		}

		float actualRadius = sqrtf((dx*dx) + (dy*dy));
		float degrees = static_cast<float>(angle * 180 / M_PI);
		if (degrees > 360) {
			degrees -= 360;
		}

		MeasureTime(degrees);
		float heading = degrees + 180; // point the camera towards the center of the circle.

		float correction = (radius - actualRadius) / radius;
		if (previousCorrection != 0)
		{
			if (fabs(correction) > fabs(previousCorrection)) {
				if (correctionFactor < 30) {
					correctionFactor++;
				}
			}
			else if (correctionFactor > 0) {
				correctionFactor--;
			}
		}
		previousCorrection = correction;
		correction *= correctionFactor;

		float tangent = static_cast<float>(angle + M_PI_2 - (correction * M_PI / 180));

		double newvx = orbitSpeed * cos(tangent);
		double newvy = orbitSpeed * sin(tangent);

		// interpoloate the speed ramp up time over 2 seconds from start time
		if (orbitSpeed < speed) {

			long long now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			long long ms = now - startTime;
			if (ms > 5000) {
				orbitSpeed = speed;
			}
			else {
				double percentage = static_cast<double>(ms) / 5000.0; // 5 second target
				orbitSpeed = static_cast<float>(percentage * speed);
				//printf("speeding up to %f\n", orbitSpeed);
			}
		}

		MoveAltHold(static_cast<float>(newvx), static_cast<float>(newvy), cz, heading);

		if (logViewer.get() != nullptr)
		{
			// monitor the sin curves so we can see how on track or off track it actually is.
			// the shape of the curve will also tell us if we are progressing at a consistent
			// speed, the more deformed the sin curve the worse our progress.

			// pack this tracking info into mavlink_vicon_position_estimate_t just because we can....

			MavLinkViconPositionEstimate est;
			est.pitch = est.roll = est.z = est.yaw = 0;
			est.usec = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

			est.x = sin(angle) * radius;
			est.y = sin(angle) * actualRadius;

			logViewer->sendMessage(est);
		}
	}
}


void OrbitCommand::MeasureTime(float degrees)
{
	if (startTime == 0)
	{
		startAngle = degrees;
		startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	}
	else if (!halfWay && fabs(startAngle - degrees) >= 180)
	{
		halfWay = true;
	}
	else if (halfWay && degrees < previousAngle)
	{
		// degrees just flipped from 359 to 0.
		auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		auto durationMs = endTime - startTime;
		endTime = endTime;
		printf("Completed orbit %d is %f seconds\n", orbits++, (static_cast<float>(durationMs) / 1000.0f));
		startTime = endTime;
		halfWay = false;
	}
	previousAngle = degrees;
}


bool RotateCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "rotate") {

			if (args.size() > 1) {
				std::string arg = args[1];
				cmd_ = arg;

				// this enables us to test what happens when offboard control is lost and resumed.
				if (arg == "resume" || arg == "pause")
				{
					return true;
				}
				speed_ = static_cast<float>(atof(arg.c_str()));
			}
			else 
			{
				speed_ = 10;
			}
			return true;
		}
	}


	return false;
}

void RotateCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	if (!com->isLocalControlSupported()) {
		throw std::runtime_error(Utils::stringf("Your drone does not support the MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED capability."));
	}

	if (cmd_ == "resume")
	{
		this->Resume();
	}
	else if (cmd_ == "pause")
	{
		this->Pause();
	}
	else
	{
		start_ = false;
		printf("rotating at current position at %f degrees/second\n", static_cast<float>(speed_));
		GotoCommand::Execute(com);
	}
}

void RotateCommand::HasLocalPosition()
{
	if (!start_) {
		// in case we are flying fast, we first do a Goto to get to a fixed stationary position before we try and start rotating.
		Goto(x, y, z, -1, yaw);
	}
}

void RotateCommand::TargetReached()
{
	count_down_ = 10;
	start_ = true;
}

void RotateCommand::UpdateTarget()
{
	if (paused) {
		return;
	}

	if (start_) {
		if (count_down_ == 0) {
			Move((tx - x) / 10, (ty - y) / 10, (tz - z) / 10, speed_, false);
		}
		else {
			count_down_--;
		}
	}
}



bool SquareCommand::Parse(const std::vector<std::string>& args)
{
    if (args.size() > 0) {
        std::string cmd = args[0];
        if (cmd == "square") {
            length_ = 1;
            speed_ = 0.2f;
            if (args.size() > 1) {
                length_ = static_cast<float>(atof(args[1].c_str()));
                if (length_ < 0 || length_ > 100) {
                    printf("invalid length '%f', expecting 1 < length < 100 ", length_);
                    return false;
                }
            } 
            if (args.size() > 2) {
                speed_ = static_cast<float>(atof(args[2].c_str()));
                if (speed_ < 0.1 || speed_ > 10) {
                    printf("invalid speed '%f', expecting 0.1 < speed < 10", speed_);
                    return false;
                }
            }
            return true;
        }
    }


    return false;
}

void SquareCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
    if (!com->isLocalControlSupported()) {
        throw std::runtime_error(Utils::stringf("Your drone does not support the MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED capability."));
    }
    started_ = false;
    leg_ = 0;	
    near = speed_;
    printf("executing a square pattern of length %f and at %f m/s\n", length_, speed_);
    GotoCommand::Execute(com);
}

void SquareCommand::setNextTarget() {
    
    switch (leg_) {
    case 0:
        printf("Driving north\n");
        tx = sx_ + length_;
        ty = sy_;
        tz = sz_;
        theading = 0;
        break;
    case 1:
        printf("Driving east\n");
        tx = sx_ + length_;
        ty = sy_ + length_;
        tz = sz_;
        theading = 0;
        break;
    case 2:
        printf("Driving south\n");
        tx = sx_;
        ty = sy_ + length_;
        tz = sz_;
        theading = 0;
        break;
    case 3:
        printf("Driving west\n");
        tx = sx_;
        ty = sy_ ;
        tz = sz_;
        tvx = 0;
        theading = 0;
        break;
    default:
        break;
    }


}

void SquareCommand::HasLocalPosition()
{
    if (!started_) {
		theading = 0;
        sx_ = x; sy_ = y; sz_ = z;
        started_ = true;
        // ok, now we can start moving by velocity
        setNextTarget();
    }
}

void SquareCommand::UpdateTarget()
{
    if (started_) {
        float dx = tx - x;
        float dy = ty - y;
        float dist = sqrtf((dx*dx) + (dy*dy));

        if (fabsf(dx) < near && fabsf(dy) < near)
        {
            leg_++;
            if (leg_ == 4) leg_ = 0;
            setNextTarget();

            // recompute to new target.
            dx = tx - x;
            dy = ty - y;
            dist = sqrtf((dx*dx) + (dy*dy));

        }

        if (dist > speed_) {
            float scale = speed_ / dist;
            dx *= scale;
            dy *= scale;
        }
        tvx = dx;
        tvy = dy;
        tvz = 0;

        MoveAltHold(tvx, tvy, tz, theading, true);
    }
}

bool WiggleCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		wiggle_size_ = 1;
		std::string cmd = args[0];
		if (cmd == "wiggle") {
            xaxis_ = false;
            wiggle_size_ = 2;
            wiggle_angle_ = 10;

			if (args.size() > 1)
			{
				wiggle_size_ = static_cast<float>(atof(args[1].c_str()));
			}
			if (args.size() > 2)
			{
				wiggle_angle_ = static_cast<float>(atof(args[2].c_str()));
			}
            if (args.size() > 3)
            {
                auto axis = args[3];
                if (axis == "x") {
                    xaxis_ = true;
                }
            }
			
			return true;
		}
	}
	return false;
}

void WiggleCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	if (!com->isAttitudeControlSupported()) {
		throw std::runtime_error(Utils::stringf("Your drone does not support the MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET capability."));
	}
	started_ = false;

	auto state = com->getVehicleState();
	auto pos = state.local_est.pos;
	this->sx_ = pos.x;
	this->sy_ = pos.y;
	this->sz_ = pos.z;
	Command::Execute(com); // subscribe.
	com->setMessageInterval(static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_LOCAL_POSITION_NED), 30);
	com->setMessageInterval(static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_ATTITUDE_TARGET), 30);

	com->requestControl();

    meter_.reset();

	// start by moving right with 10 degree roll.    
    targetAngle_ = wiggle_angle_;
	ready_ = false;
	started_ = true;
}

void WiggleCommand::HandleMessage(const MavLinkMessage& message)
{
	if (!started_)
	{
		// haven't started yet.
		return;
	}

	switch (message.msgid)
	{
	case MavLinkAttitudeTarget::kMessageId:
	{
		_current.decode(message);
		if (!ready_) {
			ready_ = true;
			flipped_ = true;
			previous_ = 0;
			ramp_up_speed_ = true;
			start_thrust_ = _current.thrust;
			DebugOutput("start_thrust_=%f", start_thrust_);
			// these PID values were calculated experimentally using AltHoldCommand, this provides the best
			// control over thrust to achieve minimal over/under shoot in a reasonable amount of time.
			thrust_controller_.setPoint(-this->sz_, .05f, .005f, 0.09f);
		}
		break;
	}
	case MavLinkLocalPositionNed::kMessageId:
	{
		if (!ready_) {
			return;
		}
		MavLinkLocalPositionNed pos;
		pos.decode(message);

        if (xaxis_) {
            wiggleX(pos);
        }
        else {
            wiggleY(pos);
        }

        meter_.reportMessageRate();

		break;
	}
	default:
		break;
	}
}

void WiggleCommand::wiggleX(const MavLinkLocalPositionNed& pos)
{
    // track how our actual pitch is coming along compared to our target 
    float roll, pitch;

    // and check position
    double dx = this->sx_ - pos.x;
    double dy = this->sy_ - pos.y;
    double z = pos.z;

    // the amount of pitch should depend on our speed in that direction.
    double speed = fabs(pos.vx);

    float ctrl = thrust_controller_.control(static_cast<float>(-z));

    float thrust = start_thrust_ + ctrl;

    // passed the midpoint.
    if ((previous_ > 0 && pos.vx < 0) || (previous_ < 0 && pos.vx > 0))
    {
        DebugOutput("wiggle velocity flipped");
        flipped_ = true;
    }
    previous_ = pos.vx;

    // fade out the pitch as we pick up speed so we don't overshoot.
    pitch = targetAngle_;
    if (flipped_ && !ramp_up_speed_) {
        float f = fmax(static_cast<float>(speed * 2), 1.0f);
        pitch /= f;
    }
    if (ramp_up_speed_ && speed > 0.5f) {
        ramp_up_speed_ = false;
    }

    // see if we just crossed the wiggle distance threshold.
    // (pitch affects the x-position).
    if (flipped_ && targetAngle_ > 0 && dx > wiggle_size_)
    {
        flipped_ = false;
        // reverse direction with a -30 degrees quick stop
        targetAngle_ = static_cast<float>(-wiggle_angle_);
        pitch = targetAngle_;
        DebugOutput("wiggle reversing direction");
    }
    else if (flipped_ && targetAngle_ < 0 && dx < -wiggle_size_)
    {
        flipped_ = false;
        // reverse direction with a 30 degrees quick stop
        targetAngle_ = static_cast<float>(wiggle_angle_);
        pitch = targetAngle_;
        DebugOutput("wiggle reversing direction");
    }

    // try and keep y on target by using roll, but only use a little bit since it shouldn't wander
    // too much in that direction.
    roll = fmax(-0.2f, fmin(0.2f, static_cast<float>((-pos.vy / 5.0f) + (dy / 10.0f))));

    //DebugOutput("ctrl=%f, sz=%f, z=%f, dz=%f, new thrust=%f", ctrl, sz_, z, sz_ - z, thrust);
    roll = static_cast<float>(roll * 180.0f / M_PI);
    vehicle->moveByAttitude(roll, pitch, 0.0f, 0, 0, 0, thrust);

}
void WiggleCommand::wiggleY(const MavLinkLocalPositionNed& pos)
{

    // track how our actual roll is coming along compared to our target 
    float roll, pitch;

    // and check position
    double dx = this->sx_ - pos.x;
    double dy = this->sy_ - pos.y;
    double z = pos.z;

    // the amount of roll should depend on our speed in that direction.
    double speed = fabs(pos.vy);

    float ctrl = thrust_controller_.control(static_cast<float>(-z));

    float thrust = start_thrust_ + ctrl;

    // passed the midpoint.
    if ((previous_ > 0 && pos.vy < 0) || (previous_ < 0 && pos.vy > 0))
    {
        DebugOutput("wiggle velocity flipped");
        flipped_ = true;
    }
    previous_ = pos.vy;

    // fade out the roll as we pick up speed so we don't overshoot.
    roll = targetAngle_;
    if (flipped_ && !ramp_up_speed_) {
        float f = fmax(static_cast<float>(speed * 2), 1.0f);
        roll /= f;
    }
    if (ramp_up_speed_ && speed > 0.5f) {
        ramp_up_speed_ = false;
    }

    // see if we just crossed the wiggle distance threshold.
    // (roll affects the y-position).
    if (flipped_ && targetAngle_ > 0 && dy < -wiggle_size_)
    {
        flipped_ = false;
        // reverse direction with a -30 degrees quick stop
        targetAngle_ = static_cast<float>(-wiggle_angle_);
        roll = targetAngle_;
        DebugOutput("wiggle reversing direction");
    }
    else if (flipped_ && targetAngle_ < 0 && dy > wiggle_size_)
    {
        flipped_ = false;
        // reverse direction with a 30 degrees quick stop
        targetAngle_ = static_cast<float>(wiggle_angle_);
        roll = targetAngle_;
        DebugOutput("wiggle reversing direction");
    }

    // try and keep x on target by using pitch, but only use a little bit since it shouldn't wander
    // too much in that direction.
    pitch = fmax(-0.2f, fmin(0.2f, static_cast<float>((pos.vx / 5.0f) - (dx / 10.0f))));

    //DebugOutput("ctrl=%f, sz=%f, z=%f, dz=%f, new thrust=%f", ctrl, sz_, z, sz_ - z, thrust);
    pitch = static_cast<float>(pitch * 180.0f / M_PI);
    vehicle->moveByAttitude(roll, pitch, 0.0f, 0, 0, 0, thrust);

}

void WiggleCommand::Close() 
{
	started_ = false;
	ready_ = false;
	if (vehicle != nullptr) {
		vehicle->releaseControl();
	}
	Command::Close();
}


// for testing PID controller.
//class AltHoldCommand : public Command
//{
//	std::shared_ptr<MavLinkVehicle> channel;
//	float sx_, sy_, sz_;
//	MavLinkAttitudeTarget _current;
//	PidController thrust_controller_;
//public:
bool AltHoldCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string cmd = args[0];
		if (cmd == "hold") {
			if (args.size() > 1)
			{
				sz_ = static_cast<float>(atof(args[1].c_str()));
			}
			else 
			{
				printf("hold - missing altitude parameter.\n");
				return false;
			}
			kp_ = 1;
			ki_ = 0;
			kd_ = 0;
			if (args.size() > 2)
			{
				kp_ = static_cast<float>(atof(args[2].c_str()));
			}
			if (args.size() > 3)
			{
				ki_ = static_cast<float>(atof(args[3].c_str()));
			}
			if (args.size() > 4)
			{
				kd_ = static_cast<float>(atof(args[4].c_str()));
			}
			return true;
		}
	}
	return false;
}

void AltHoldCommand::Close()
{
	if (vehicle != nullptr) {
		vehicle->releaseControl();
	}
	Command::Close();
}

void AltHoldCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	vehicle = com;
	if (!com->isAttitudeControlSupported()) {
		throw std::runtime_error(Utils::stringf("Your drone does not support the MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET capability."));
	}
	ready_ = false;
	started_ = false;
	auto state = com->getVehicleState();
	auto pos = state.local_est.pos;
	this->sx_ = pos.x;
	this->sy_ = pos.y;

	// this->sz_ = pos.z; // user defined target.
	Command::Execute(com); // subscribe.
	com->setMessageInterval(static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_LOCAL_POSITION_NED), 30);
	com->setMessageInterval(static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_ATTITUDE_TARGET), 30);

	com->requestControl();

	// move to local position keeps the offboard control happy.
	com->moveToLocalPosition(pos.x, pos.y, pos.z, true, static_cast<float>(state.attitude.yaw * M_PI / 180));

	started_ = true;
}

void AltHoldCommand::HandleMessage(const MavLinkMessage& message)
{
	if (!started_)
	{
		// haven't started yet.
		return;
	}

	switch (message.msgid)
	{
	case MavLinkAttitudeTarget::kMessageId:
	{
		_current.decode(message);
		if (!ready_) {
			start_thrust_ = _current.thrust;
			DebugOutput("start_thrust_=%f", start_thrust_);
			thrust_controller_.setPoint(this->sz_, kp_, ki_, kd_);
			ready_ = true;
		}
		break;
	}
	case MavLinkLocalPositionNed::kMessageId:
	{
		if (!ready_) {
			return;
		}
		MavLinkLocalPositionNed pos;
		pos.decode(message);

		// and check position
		//double dx = this->sx_ - pos.x;
		//double dy = this->sy_ - pos.y;
		double z = pos.z; 

		float ctrl = thrust_controller_.control(static_cast<float>(z));
		float thrust = start_thrust_ + ctrl;


		printf("thrust:%f\tctrl:%f\n", thrust, ctrl);


		// try and keep x & y on target by using pitch & roll, but only use a little bit since it shouldn't wander
		// too much in that direction.
		float pitch = fmax(-0.2f, fmin(0.2f, pos.vx / 5.0f));
		float roll = fmax(-0.2f, fmin(0.2f, -pos.vy / 5.0f));

		// adjust thrust so we keep steady height target 
		thrust = static_cast<float>(fmax(0.01, fmin(1.0, thrust)));

		//DebugOutput("ctrl=%f, sz=%f, z=%f, dz=%f, new thrust=%f", ctrl, sz_, z, sz_ - z, thrust);
		vehicle->moveByAttitude(roll, pitch, 0, 9, 0, 0, thrust);
		break;
	}
	default:
		break;
	}
}

std::string replaceAll(std::string s, char toFind, char toReplace) {
	size_t pos = s.find_first_of(toFind, 0);
	while (pos != std::string::npos) {
		s.replace(pos, 1, 1, toReplace);
		pos = s.find_first_of(toFind, 0);
	}
	return s;
}

std::string normalize(std::string arg) {
	if (FileSystem::kPathSeparator == '\\') {
		return replaceAll(arg, '/', '\\'); // make sure user input matches what FileSystem will do when resolving paths.
	}
	return arg;
}

std::string toPX4Path(std::string arg) {
	if (FileSystem::kPathSeparator == '\\') {
		return replaceAll(arg, '\\', '/'); // PX4 uses '/'
	}
	return arg;
}

bool FtpCommand::Parse(const std::vector<std::string>& args)
{
	//	"ftp [ls|cd name|get source [target]|put source target]";
	cmd = none;
	if (args.size() > 0) {
		std::string command = args[0];
		if (command == "ls") {
			cmd = list;

			if (args.size() > 1) {
				std::string rel = normalize(args[1]);
				source = FileSystem::resolve(cwd, rel);
			}
			else {
				source = FileSystem::resolve(cwd, "");
			}
		}
		else if (command == "cd") {
			cmd = cd;
			if (args.size() > 1) {
				std::string rel = normalize(args[1]);
				cwd = FileSystem::resolve(cwd, rel);
			}
			else {
				cmd = none;
				printf("current directory is %s\n", cwd.c_str());
			}
		}
		else if (command == "get") {
			cmd = get;
			if (args.size() > 1) {
				std::string rel = normalize(args[1]);
				target = FileSystem::resolve(cwd, rel);
				if (args.size() > 2) {
					source = args[2];
				}
				else {
					cmd = none;
					printf("Missing local file name\n");
				}
			}
			else {
				cmd = none;
				printf("Missing remote file name\n");
			}
		}
		else if (command == "put") {
			cmd = put;
			if (args.size() > 1) {
				// local remote
				source = args[1];
				if (args.size() > 2) {
					std::string rel = normalize(args[1]);
					target = FileSystem::resolve(cwd, rel);
				}
				else {
					cmd = none;
					printf("Missing remote file name\n");
				}
			}
			else {
				cmd = none;
				printf("Missing local file name\n");
			}
		}
		else if (command == "rm") {
			cmd = remove;
			if (args.size() > 1) {
				std::string rel = normalize(args[1]);
				target = FileSystem::resolve(cwd, rel);
				target = toPX4Path(target);
			}
			else {
				cmd = none;
				printf("Missing remote file name\n");
			}
		}
	}
	return (cmd != none);
}


void FtpCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	if (client == nullptr) {
		client = std::make_shared<MavLinkFtpClient>(166, 1);
		client->connect(com->getConnection());
	}
	if (!client->isSupported()) {
		printf("ftp commands are not supported by your drone\n");
	}
	switch (cmd)
	{
	case FtpCommand::list:
		doList();
		break;
	case FtpCommand::cd:
		// already handled by the parse method.
		break;
	case FtpCommand::get:
		doGet();
		break;
	case FtpCommand::put:
		doPut();
		break;
	case FtpCommand::remove:
		doRemove();
		break;
	default:
		break;
	}
}

void FtpCommand::startMonitor()
{
	stopMonitor();
	progress.complete = false;
	progress.current = 0;
	progress.goal = 0;
	progress.message = "";
	monitorThread = std::thread{ &FtpCommand::monitor, this };
}

void FtpCommand::stopMonitor() {

	if (monitorThread.joinable()) {
		progress.complete = true;
		monitorThread.join();
	}
}

void FtpCommand::Close() {
	Command::Close();
	if (client != nullptr) {
		client->cancel();
	}
}


void FtpCommand::doList() {

	std::vector<MavLinkFileInfo> files;

	startMonitor();
	client->list(progress, toPX4Path(source), files);
	stopMonitor();
	printf("\n");

	int count = 0;
	for (auto iter = files.begin(); iter != files.end(); iter++)
	{
		count++;
		auto info = *iter;
		if (info.is_directory) {
			printf("<DIR>    %s\n", info.name.c_str());
		}
		else {
			printf("%8d %s\n", info.size, info.name.c_str());
		}
	}
	if (count == 0) {
		printf("directory is empty\n");
	}
	if (progress.error != 0) {
		printf("%s\n", progress.message.c_str());
	}
	if (progress.average_rate != 0) {
		printf("%d msgs received, %f milliseconds per packet, longest delay=%f\n", progress.message_count, progress.average_rate, progress.longest_delay);
	}
}

void FtpCommand::doGet() {
	
	std::string fsTarget = normalize(target);
	std::string leaf = FileSystem::getFileName(fsTarget);
	if (leaf.size() > 0 && leaf[0] == '/' || leaf[0] == '\\'){
		leaf = leaf.substr(1);
	}
	bool wildcards;
	if (!parse(leaf, wildcards)) {
		printf("wildcard pattern '%s' is too complex\n", leaf.c_str());
	} else if (wildcards) {
		std::vector<MavLinkFileInfo> files;		
		FileSystem::removeLeaf(fsTarget);
		std::string dir = toPX4Path(fsTarget);
		printf("getting all files in: %s matching '%s' ", dir.c_str(), leaf.c_str());

		startMonitor();
		client->list(progress, toPX4Path(dir), files);
		stopMonitor();
		printf("\n");

		int count = 0;
		int matching = 0;
		for (auto iter = files.begin(); iter != files.end(); iter++)
		{
			count++;
			auto info = *iter;
			if (!info.is_directory) {
				if (matches(leaf, info.name))
				{
					matching++;
					printf("Getting %8d %s", info.size, info.name.c_str());
					std::string sourceFile = FileSystem::combine(source, info.name);
					std::string targetFile = FileSystem::combine(dir, info.name);
					startMonitor();
					client->get(progress, toPX4Path(targetFile), sourceFile);
					stopMonitor();
					printf("\n");
					if (progress.error != 0) {
						break;
					}
				}
			}
		}
		if (progress.error == 0) {
			if (count == 0) {
				printf("directory is empty\n");
			}
			else if (matching == 0) {
				printf("no matching files\n");
			}
		}

	}
	else {
		printf("Getting %s", target.c_str());
		startMonitor();
		client->get(progress, toPX4Path(target), source);
		stopMonitor();
		printf("\n");
	}
	if (progress.error != 0) {
		printf("%s\n", progress.message.c_str());
		printf("get failed\n");
	}
	else {
		printf("ok\n");
	}
	if (progress.average_rate != 0) {
		printf("%d msgs received, %f milliseconds per packet, longest delay=%f\n", progress.message_count, progress.average_rate, progress.longest_delay);
	}
}

void FtpCommand::doPut() {
	startMonitor();
	printf("Writing %s", target.c_str());
	client->put(progress, toPX4Path(target), source);
	stopMonitor();
	printf("\n");
	if (progress.error != 0) {
		printf("%s\n", progress.message.c_str());
		printf("put failed\n");
	}
	else {
		printf("ok\n");
	}
	if (progress.average_rate != 0) {
		printf("%d msgs received, %f milliseconds per packet, longest delay=%f\n", progress.message_count, progress.average_rate, progress.longest_delay);
	}
}

void FtpCommand::doRemove() {
	std::string fsTarget = normalize(target);
	std::string leaf = FileSystem::getFileName(fsTarget);
	if (leaf.size() > 0 && leaf[0] == '/' || leaf[0] == '\\') {
		leaf = leaf.substr(1);
	}
	bool wildcards = false;
	if (!parse(leaf, wildcards)) {
		printf("wildcard pattern '%s' is too complex\n", leaf.c_str());
	} else if (wildcards) {
		std::vector<MavLinkFileInfo> files;
		FileSystem::removeLeaf(fsTarget);
		std::string dir = toPX4Path(fsTarget);
		printf("removing all files in %s matching '%s'\n", dir.c_str(), leaf.c_str());
		startMonitor();
		client->list(progress, toPX4Path(dir), files);
		stopMonitor();
		printf("\n");

		int count = 0;
		int matching = 0;
		for (auto iter = files.begin(); iter != files.end(); iter++)
		{
			count++;
			auto info = *iter;
			if (!info.is_directory) {
				if (matches(leaf, info.name)) {
					matching++;
					printf("Removing %s ", info.name.c_str());
					std::string targetFile = FileSystem::combine(normalize(dir), info.name); 
					startMonitor();
					client->remove(progress, toPX4Path(targetFile));
					stopMonitor();
					printf("\n");
					if (progress.error != 0) {
						break;
					}
				}
			}
		}
		if (progress.error == 0) {
			if (count == 0) {
				printf("directory is empty\n");
			}
			else if (matching == 0) {
				printf("no matching files\n");
			}
		}

	}
	else {
		printf("Removing %s ", target.c_str());
		startMonitor();
		client->remove(progress, toPX4Path(target));
		stopMonitor();
		printf("\n");
	}

	if (progress.error != 0) {
		printf("remove failed\n");
	}
	else {
		printf("ok\n");
	}
}

void FtpCommand::monitor() {
	std::string msg = progress.message;
	double percent = 0;

	while (!progress.complete) {
		if (progress.goal > 0) {
			double p = static_cast<double>(progress.current) / static_cast<double>(progress.goal);
			if (static_cast<int>(p * 10) > static_cast<int>(percent * 10)) {
				printf(".");
				percent = p;
			}
			if (progress.message != "" && progress.message != msg) {
				msg = progress.message;
				printf("%s\n", msg.c_str());
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}


bool FtpCommand::parse(const std::string& name, bool& wildcards) const
{
	// we only support very simple patterns for now.
	// each wildcard must be separated by literal.
	wildcards = false;
	size_t wildcardpos = -1;
	for (size_t i = 0, s = name.size(); i < s; i++)
	{
		char ch = name[i];
		if (ch == '*' || ch == '?') {
			if (wildcards && wildcardpos + 1 == i) {
				// back to back wildcards with no literal in between is too complex.
				return false;
			}
			wildcards = true;
			wildcardpos = i;
		}
	}
	return true;
}

bool FtpCommand::matches(const std::string& pattern, const std::string& name) const
{
	size_t p = 0;
	size_t ps = pattern.size();
	// we only support simple matching for now, we can add full regex later if we need it.
	for (size_t i = 0, s = name.size(); i < s; i++)
	{
		if (p >= ps) {
			return false;
		}
		char pc = pattern[p];
		char ch = name[i];
		if (pc == '?' || pc == ch) {
			// yep!
			p++;
		}
		else if (pc != '*') {
			return false;
		}
		else if (p + 1 < ps && pattern[p + 1] == ch) {
			// '*' is done we found the next matching char
			p += 2;
		}
	}
	if (p + 1 == ps && pattern[p] == '*') {
		// this is ok.
		return true;
	}
	return p == ps;
}

bool NshCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string command = args[0];
		if (command == "nsh") {
			return true;
		}
	}
	return false;
}

void NshCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	Command::Execute(com);

	std::string line = "\n";
	send(line);

	while (!std::cin.eof()) {
		std::getline(std::cin, line);
		if (line == "x" || line == "quit" || line == "bye" || line == "exit") {
			return;
		}
		line += '\n';
		send(line);
	}
}

void NshCommand::send(std::string& msg)
{
	MavLinkSerialControl ctrl;
	ctrl.device = static_cast<uint8_t>(SERIAL_CONTROL_DEV::SERIAL_CONTROL_DEV_SHELL);
	ctrl.flags = static_cast<uint8_t>(SERIAL_CONTROL_FLAG::SERIAL_CONTROL_FLAG_RESPOND) | 
                 static_cast<uint8_t>(SERIAL_CONTROL_FLAG::SERIAL_CONTROL_FLAG_EXCLUSIVE);
	ctrl.baudrate = 0;
	ctrl.timeout = 0;
	int len = static_cast<int>(msg.size());
	if (len > 70) len = 70;
	ctrl.count = static_cast<uint8_t>(len);
	::memcpy(ctrl.data, msg.c_str(), len);
	vehicle->sendMessage(ctrl);
}

void NshCommand::Close()
{
	MavLinkSerialControl ctrl;
	ctrl.device = 0;
	ctrl.flags = 0;
	ctrl.baudrate = 0;
	ctrl.timeout = 0;
	ctrl.count = 0;
	if (vehicle != nullptr) {
		vehicle->sendMessage(ctrl);
	}
}

void NshCommand::HandleMessage(const MavLinkMessage& msg)
{
	if (msg.msgid == static_cast<uint8_t>(MavLinkMessageIds::MAVLINK_MSG_ID_SERIAL_CONTROL)) {

		MavLinkSerialControl ctrl;
		ctrl.decode(msg);
		int len = ctrl.count;
		if (len > 0)
		{
			if (len > 3 && ctrl.data[len - 1] == 'K' && ctrl.data[len - 2] == '[' && ctrl.data[len - 3] == '\x1b') {
				// this is an ERASE_END_LINE command which we ignore.
				len -= 3;
			}
			std::cout.write(reinterpret_cast<const char*>(&ctrl.data[0]), len);
		}
	}
}



bool SetMessageIntervalCommand::Parse(const std::vector<std::string>& args)
{
	msgid_ = 0;
	frequency_ = 0;

	if (args.size() == 3) {
		std::string cmd = args[0];
		cmd = Utils::toLower(cmd);
		if (cmd == "setmessageinterval") {
			msgid_ = atoi(args[1].c_str());
			frequency_ = atoi(args[2].c_str());		
			if (msgid_ <= 0) {
				printf("invalid message id %d.\n", msgid_);
				return false;
			}
			if (frequency_ < 0 || frequency_ > 1000) {
				printf("invalid frequence %d, valid range is 0 <= f <= 1000.\n", frequency_);
				return false;
			}
			return true;
		}
	}
	return false;
}

void SetMessageIntervalCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	com->setMessageInterval(msgid_, frequency_);
}

bool WaitForAltitudeCommand::Parse(const std::vector<std::string>& args)
{
	if (args.size() > 0) {
		std::string cmd = args[0];
		cmd = Utils::toLower(cmd);
		if (cmd == "waitforaltitude") {
			if (args.size() == 4) {
				z = static_cast<float>(atof(args[1].c_str()));
				dz = static_cast<float>(atof(args[2].c_str()));
				dvz = static_cast<float>(atof(args[3].c_str()));
				if (z > 0) {
					z = -z;
				}
				return true;
			}
			else {
				printf("wrong number of arguments.\n");
				PrintHelp();
				return false;
			}
		}
	}
	return false;
}

void WaitForAltitudeCommand::Execute(std::shared_ptr<MavLinkVehicle> com)
{
	bool rc = false;
	if (com->waitForAltitude(z, dz, dvz).wait(30000, &rc)) {
		if (!rc) {
			printf("altitude not reached\n");
		}
		else {
			printf("target altitude & velocity reached\n");
		}
	}
	else {
		printf("timeout waiting for set altitude\n");
	}

}

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef COMMNADS_H
#define COMMNADS_H

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "MavLinkVehicle.hpp"
#include "MavLinkVideoStream.hpp"
#include "MavLinkFtpClient.hpp"
#include "Utils.hpp"
#include <memory>

using namespace mavlinkcom;

class Command
{
	int subscription = 0;
protected:
	std::shared_ptr<MavLinkVehicle> vehicle;
public:
	Command();
	~Command();

	std::string Name;

	virtual void Close();
	virtual bool Parse(std::vector<std::string>& args) = 0;
	virtual void PrintHelp() = 0;
	// you must call this method if you want HandleMessage to be called subsequently.
	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
	virtual void HandleMessage(const MavLinkMessage& msg) {}
};

class ArmDisarmCommand : public Command
{
	bool arm;
public:
	ArmDisarmCommand() {
		this->Name = "arm|disarm";
		this->arm = false;
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("arm|disam - arms or disarms the drone\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
};


class GetParamsCommand : public Command
{
	FILE* ptr;
public:
	GetParamsCommand() {
		this->Name = "params [logfile] [set name value] [get name]";
		this->ptr = NULL;
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("params [logfile]- fetches the PX4 parameters and prints them or writes them to a file.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void Close()
	{
		if (ptr != NULL)
		{
			fclose(ptr);
			ptr = NULL;
		}
	}

private:
	void OpenLog(const char* fileName)
	{
#ifdef _WIN32
		int rc = fopen_s(&ptr, fileName, "w");
		if (rc != 0)
		{
			printf("Error opening file '%s', code=%d\n", fileName, rc);
		}
#else
		ptr = fopen(fileName, "w");
#endif
	}

};

class GetSetParamCommand : public Command
{
    std::string name;
    float value;
    bool get = false;
public:
    GetSetParamCommand() {
        this->Name = "param [set name value] [get name]";
    }
	virtual bool Parse(std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("param [set name value] [get name]  -- get or set a parameter by name (use params to see the list).\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
};


class TakeOffCommand : public Command
{
	float altitude = 0;
	float targetAlt;
	bool reached;
	bool offground;
	const float delta = 1;
public:
	TakeOffCommand() {
		this->Name = "takeoff alt";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("takeoff alt - take off to given altitude above ground (in meters).\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HandleMessage(const MavLinkMessage& msg);

};


class LandCommand : public Command
{
	bool landed;
public:
	LandCommand() {
		this->Name = "land";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("land  - auto-land the drone at the current location.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HandleMessage(const MavLinkMessage& msg);

};


class RtlCommand : public Command
{
	bool landed;
public:
	RtlCommand() {
		this->Name = "rtl";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("rtl  - fly home (return to launch point).\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HandleMessage(const MavLinkMessage& msg);

};


class LoiterCommand : public Command
{
public:
    LoiterCommand() {
        this->Name = "loiter";
    }
	virtual bool Parse(std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("loiter  - stay at current position.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

};

class RequestImageCommand : public Command
{
public:
	RequestImageCommand() {
		this->Name = "req_img";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("req_img  - request image.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
	virtual void HandleMessage(const MavLinkMessage& msg);

private:
	std::shared_ptr<MavLinkVideoClient> stream;
};

class MissionCommand : public Command
{
	bool landed;
public:
	MissionCommand() {
		this->Name = "mission";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("mission  - fly preprogrammed mavlink mission.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HandleMessage(const MavLinkMessage& msg);

};


class CapabilitiesCommand : public Command
{
	bool landed;
public:
	CapabilitiesCommand() {
		this->Name = "cap";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("cap  - fetch and print drone capabilities.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

};

class StatusCommand : public Command
{
	bool printStatus = false;
	bool printExtStatus = false;
	bool printHeartbeat = false;
	bool printHomePosition = false;
public:
	StatusCommand() {
		this->Name = "status";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("status  - print next SYS_STATUS message.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HandleMessage(const MavLinkMessage& msg);

};

class PositionCommand : public Command
{
	bool printLocalPosition = false;
	bool printGlobalosition = false;
	bool setHome = false;
public:
	PositionCommand() {
		this->Name = "pos [home]";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("pos [home] - print current local and global position and optionally set this as the home position.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HandleMessage(const MavLinkMessage& msg);

};

class SendImageCommand : public Command
{
	std::shared_ptr<MavLinkNode> logViewer;
	std::string fileName;
	int width;
	int height;

public:
	SendImageCommand() {
		this->Name = "sendimage filename width height";
	}
	void setLogViewer(std::shared_ptr<MavLinkNode> logViewerProxy) {
		logViewer = logViewerProxy;
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("sendimage filename width height - send the specified image to remote proxy.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

};


class GotoCommand : public Command
{
	bool hasLocalPosition;
	bool requestedControl;
	bool hasControl;
	bool targetReached;
	bool targetPosition;
	bool targetVelocity;	
	bool targetVelocityAltHold;
	bool settled; // after target reached.
	
protected:
	std::shared_ptr<MavLinkVehicle> channel;
	float x, y, z; // current
	float tx, ty, tz; // target position
	float vx, vy, vz; // current speed
	
	// current attitude
	float pitch;
	float pitchSpeed;
	float roll;
	float rollSpeed;
	float yaw;
	float yawSpeed;

	// targets for planned movement.
	float tvx, tvy, tvz; // target velocities
	bool is_yaw; // is target a heading or a rate (true=heading, false=rate).
	float theading; // target heading
	float targetSpeed;
	bool paused = false;
	const float nearDelta = 0.5f; // meters
	const float almostStationery = 0.6f;
public:
	GotoCommand() {
		this->Name = "goto x,y,z";
		channel = nullptr;
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("goto x,y,z  - move the drone to local coordinates x,y,z (in meters).\n");
	}

	virtual void Close();

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void TakeControl();

	virtual void HandleMessage(const MavLinkMessage& msg);

	virtual void HasLocalPosition();

    virtual void UpdateTarget();

	// goto target location
	virtual void Goto(float x, float y, float z, float speed, float heading, bool isYaw = true);

	// start moving using the given velocities
	virtual void Move(float vx, float vy, float vz, float heading, bool isYaw = true);

	// start moving using the given velocities in x&y but with fixed altitude.
	virtual void MoveAltHold(float vx, float vy, float z, float heading, bool isYaw = true);

	virtual void TargetReached();

	void Pause() { paused = true; }
	void Resume() { paused = false; }

	virtual void OnLostOffboardControl() {
		printf("### ERROR: user took over control of the drone, we lost offboard control\n");
		Close(); // stop tracking and release control
	}
};

class OrbitCommand : public GotoCommand
{
public:
	OrbitCommand() {
		this->Name = "orbit radius [speed]";
		flyingToRadius = false;
		orbiting = false;
	}
	void setLogViewer(std::shared_ptr<MavLinkNode> logViewerProxy) {
		logViewer = logViewerProxy;
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("orbit radius [speed] - orbit the current location & altitude at the given radius (in meters) with optional speed (default 1m/s).\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HasLocalPosition();

    virtual void UpdateTarget();
private:
    void MeasureTime(float degrees);

	std::shared_ptr<MavLinkNode> logViewer;
	float radius;
	bool flyingToRadius;
	bool orbiting;
	float cx, cy, cz;
	float correctionFactor = 1;
	float previousCorrection = 0;
	float speed = 1;
	float orbitSpeed; // ramp up to requested speed
	// orbit tracking
    float startAngle;
    float previousAngle;
    bool halfWay;
    long long startTime;
    int orbits;
};

class RotateCommand : public GotoCommand
{
	float speed_;
	bool start_;
	int count_down_;
	std::string cmd_;

public:
	RotateCommand() {
		this->Name = "rotate [speed]";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("rotate [speed] - rotate the drone at current location with given degrees/s\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void HasLocalPosition();

	virtual void TargetReached();
	
	virtual void UpdateTarget();
};

class PidController
{
private:
	float set_point_;
	float kProportional_;
	float kIntegral_;
	float kDerivative_;
	float previous_error_;
	bool previous_set;
	float sum_;
	std::chrono::time_point<std::chrono::system_clock> prev_time_;
public:
	//set desired point.
	void setPoint(float newSetPoint, float kProportional, float kIntegral, float kDerivative) {
		set_point_ = newSetPoint;
		kProportional_ = kProportional;
		kIntegral_ = kIntegral;
		kDerivative_ = kDerivative;
		prev_time_ = std::chrono::system_clock::now();
		previous_error_ = 0;
		sum_ = 0;
		previous_set = false;
	}
	float control(float processVariable) {
		auto t = std::chrono::system_clock::now();
		auto diff = std::chrono::system_clock::now() - prev_time_;

		float error = set_point_ - processVariable;
		if (!previous_set)
		{
			previous_set = true;
			previous_error_ = error;
			return 0;
		}

		float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(diff).count()) * 0.000001f;
		float proportionalGain = 0;
		float derivativeGain = 0;
		float integralGain = 0;

		if (kProportional_ != 0) {
			proportionalGain = error * kProportional_;
		}
		if (kDerivative_ != 0) {
			float derivative = (error - previous_error_) / dt;
			derivativeGain = derivative * kDerivative_;
		}
		if (kIntegral_ != 0) {
			sum_ += error * dt;
			integralGain = sum_ * kIntegral_;
		}

		previous_error_ = error;
		prev_time_ = t;

		return proportionalGain + derivativeGain + integralGain;
	}
};

// for testing attitude control.
class WiggleCommand : public Command
{
	float sx_, sy_, sz_;
	float wiggle_size_ = 2;
	float wiggle_angle_ = 30;
	float targetRoll_ = 0;
	bool started_;
	bool ready_;
	float start_thrust_;
	float previous_;
	bool flipped_;
	bool ramp_up_speed_;
	MavLinkAttitudeTarget _current;
	PidController thrust_controller_;
public:
	WiggleCommand() {
		this->Name = "wiggle";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("wiggle - wiggle the drone using low level attitude control.\n");
	}
	virtual void Close();
	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
	virtual void HandleMessage(const MavLinkMessage& message);
};

class IdleCommand : public Command
{
private:
	bool requested_control_;
	bool has_control_;
public:

	IdleCommand() {
		this->Name = "idle";
	}
	virtual void PrintHelp() {
		printf("idle - tests a noop offboard control operation.\n");
	}
	virtual bool Parse(std::vector<std::string>& args);
	virtual void Close();
	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
	virtual void HandleMessage(const MavLinkMessage& message);
	virtual void OnLostOffboardControl();
};

// for testing PID controller.
class AltHoldCommand : public Command
{
	float sx_, sy_, sz_;
	float kp_, ki_, kd_;
	float start_thrust_;
	bool started_;
	bool ready_;
	MavLinkAttitudeTarget _current;
	PidController thrust_controller_;
public:
	AltHoldCommand() {
		this->Name = "hold";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("hold alt kp ki kd  - move to given altitude using given PID controls and hold there.\n");
	}
	virtual void Close();
	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
	virtual void HandleMessage(const MavLinkMessage& message);
};



class FtpCommand : public Command
{
	enum FtpCommandEnum {
		none, list, cd, get, put, remove
	};
	FtpCommandEnum cmd;
	std::string source;
	std::string target;
	std::string cwd;
	std::shared_ptr<MavLinkFtpClient> client;
public:
	FtpCommand() {
		this->Name = "ls [dir]\ncd name\nget remoteFile [localFile]\nput localFile remoteFile\nrm remoteFile]";
		this->cmd = none;
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("ftp command allows you to transfer files to/from the remote drone, if the drone supports the MAV_PROTOCOL_CAPABILITY_FTP.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

	virtual void Close();
private:
	std::string resolve(std::string name);
};

class NshCommand : public Command
{
public:
	NshCommand() {
		this->Name = "nsh - start NuttX Shell";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("nsh - start NuttX Shell.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
	virtual void Close();
	virtual void HandleMessage(const MavLinkMessage& msg);
private:
	void send(std::string& msg);
};


class SetMessageIntervalCommand : public Command
{
public:
	SetMessageIntervalCommand() {
		this->Name = "SetMessageInterval msgid interval - turn on/off a given mavlink stream";
	}
	virtual bool Parse(std::vector<std::string>& args);

	virtual void PrintHelp() {
		printf("SetMessageInterval msgid interval - turn on/off a given mavlink stream.\n");
		printf("where the msgid is an integer from mavlink common.xml, and the interval is.\n");
		printf("a frequency, the number of messages per second you'd like to receive.\n");
		printf("Setting this to 0 turns off the given message stream.\n");
	}

	virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
private:
	int msgid_;
	int frequency_;
};



#endif

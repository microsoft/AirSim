// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef COMMNADS_H
#define COMMNADS_H

#include "MavLinkVehicle.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkVideoStream.hpp"
#include "MavLinkFtpClient.hpp"
#include "Utils.hpp"
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <chrono>
#include <vector>

using namespace mavlinkcom;

// from main.cpp
void DebugOutput(const char* message, ...);

class RateMeter {
public:
    void reset() {
        start_time_ = std::chrono::system_clock::now();
        msg_count_ = 0;
    }
    void reportMessageRate() {
        auto t = std::chrono::system_clock::now();
        auto diff = t - start_time_;
        double dt = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(diff).count()) / 1000.0;
        if (dt > 1) {
            // report our message rate.
            DebugOutput("msg rate is %d Hz", msg_count_);
            start_time_ = t;
            msg_count_ = 0;
        }
        msg_count_++;
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    long msg_count_;
};

class Command
{
public:
    Command();
    ~Command();

    std::string Name;

    virtual void Close();
    virtual bool Parse(const std::vector<std::string>& args) = 0;
    virtual void PrintHelp() = 0;
    // you must call this method if you want HandleMessage to be called subsequently.
    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
    virtual void HandleMessage(const MavLinkMessage& msg) {
        unused(msg);
    }

    static std::vector<std::string> parseArgs(std::string s);
    static Command* create(const std::string& line);
    static Command* create(const std::vector<std::string>& args);
    static std::vector<Command*> const * getAllCommand();
    static void setAllCommand(std::vector<Command*> const * all_commands);
    static constexpr const char* kCommandLogPrefix = "cmd:";
protected:
    std::shared_ptr<MavLinkVehicle> vehicle;
private:
    int subscription = 0;
};

class ArmDisarmCommand : public Command
{
    bool arm;
public:
    ArmDisarmCommand() {
        this->Name = "arm|disarm";
        this->arm = false;
    }
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("cap  - fetch and print drone capabilities.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

};


class BatteryCommand : public Command
{
    bool got_battery_;
public:
    BatteryCommand() {
        this->Name = "battery";
        this->got_battery_ = false;
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("battery  - print next battery message.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
    virtual void HandleMessage(const MavLinkMessage& msg);

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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("pos [home] - print current local and global position and optionally set this as the home position.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

    virtual void HandleMessage(const MavLinkMessage& msg);

};

class FakeGpsCommand : public Command
{
    bool started = false;
    std::thread hil_thread;
    std::shared_ptr<MavLinkVehicle> com;
public:
    FakeGpsCommand() {
        this->Name = "fakegps [start|stop]";
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("fakegps [start|stop] - start stop simple generation of fake GPS input.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> mav);

    void GpsThread();

    float addNoise(float x, float scale);
    void stop();
    void start();
};

class HilCommand : public Command
{
    bool started = false;
    std::thread hil_thread;
    std::shared_ptr<MavLinkVehicle> com;
public:
    HilCommand() {
        this->Name = "hil [start|stop]";
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("hil [start|stop] - start stop simple hil simulation mode to generate fake GPS input.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> mav);

    void HilThread();

    float addNoise(float x, float scale);
    void stop();
    void start();
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
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("sendimage filename width height - send the specified image to remote proxy.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

};


class PlayLogCommand : public Command
{
public:
    PlayLogCommand() {
        this->Name = "playlog filename";		
    }

    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("playlog filename - play commands in specified .mavlink file.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

private:
    MavLinkFileLog log_;
    float quaternion_[4];
    float x, y, z;
    std::string _fileName;
    bool _syncParams;
};


class DumpLogCommandsCommand : public Command
{
public:
    DumpLogCommandsCommand() {
        this->Name = "dumplogcommands log_folder";
    }

    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("dumplogcommands log_folder - dump commands for .mavlink file found in log folder output in subfolder.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
private:
    static void processLogCommands(MavLinkFileLog& log, const std::string& out_folder);

private:
    std::string log_folder_;
};


class GotoCommand : public Command
{
    bool hasLocalPosition;
    bool requestedControl;
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
    float cruise_speed_;
    bool paused = false;
    const float nearDelta = 0.5f; // meters
    const float almostStationery = 0.6f;
public:
    GotoCommand() {
        this->Name = "goto x,y,z";
        channel = nullptr;
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("goto x y z [speed] - move the drone to local coordinates x,y,z (in meters).\n");
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
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("rotate [speed] - rotate the drone at current location with given degrees/s\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

    virtual void HasLocalPosition();

    virtual void TargetReached();
    
    virtual void UpdateTarget();
};


// this is our moveByVelocityZ test.
class SquareCommand : public GotoCommand
{

public:
    SquareCommand() {
        this->Name = "square length [speed]";
        length_ = 0;
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("square length [speed] - make a square of given length at given speed (default 0.2m/s).\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

    virtual void HasLocalPosition();

    virtual void UpdateTarget();

private:
    void setNextTarget();
    bool started_;
    int leg_;
    float length_;
    float speed_;
    float sx_, sy_, sz_; // start pos
    float near = 0.5f; // meters
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
            float derivative = dt > 0 ? (error - previous_error_) / dt : 0;
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
    float targetAngle_ = 0;
    bool started_;
    bool ready_;
    float start_thrust_;
    float previous_;
    bool flipped_;
    bool ramp_up_speed_;
    bool xaxis_;
    RateMeter meter_;
    MavLinkAttitudeTarget _current;
    PidController thrust_controller_;
    void wiggleX(const MavLinkLocalPositionNed& pos);
    void wiggleY(const MavLinkLocalPositionNed& pos);
public:
    WiggleCommand() {
        this->Name = "wiggle [distance] [angle] [direction]";
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("wiggle - wiggle the drone using low level attitude control.\n");
        printf("options:\n");
        printf("    distance - in meters to travel between direction changes.\n");
        printf("    angle    - the amount of attitude to get us moving in the specified direction.\n");
        printf("    x or y   - the direction to move.\n");
    }
    virtual void Close();
    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
    virtual void HandleMessage(const MavLinkMessage& message);
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
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("hold alt kp ki kd  - move to given altitude using given PID controls and hold there.\n");
    }
    virtual void Close();
    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
    virtual void HandleMessage(const MavLinkMessage& message);
};



class FtpCommand : public Command
{
public:
    FtpCommand() {
        this->Name = "ls [dir]\ncd name\nget remoteFile [localFile]\nput localFile remoteFile\nrm remoteFile\nmkdir remotepath\nrmdir remotepath]";
        this->cmd = none;
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("ftp command allows you to transfer files to/from the remote drone, if the drone supports the MAV_PROTOCOL_CAPABILITY_FTP.\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);

    virtual void Close();
private:

    std::string resolve(std::string name);
    void doList();
    void doGet();
    void doPut();
    void doRemove();
    void doMkdir();
    void doRmdir();
    void monitor();
    bool parse(const std::string& name, bool& wildcards) const;
    bool matches(const std::string& pattern, const std::string& name) const;
    void startMonitor();
    void stopMonitor();
    enum FtpCommandEnum {
        none, list, cd, get, put, remove, mkdir, rmdir
    };
    FtpCommandEnum cmd;
    std::string source;
    std::string target;
    std::string cwd;
    std::shared_ptr<MavLinkFtpClient> client;
    MavLinkFtpProgress progress;
    std::thread monitorThread;
};

class NshCommand : public Command
{
public:
    NshCommand() {
        this->Name = "nsh - start NuttX Shell";
    }
    virtual bool Parse(const std::vector<std::string>& args);

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
    virtual bool Parse(const std::vector<std::string>& args);

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


class WaitForAltitudeCommand : public Command
{
public:
    WaitForAltitudeCommand() {
        this->Name = "WaitForAltitude z dz dvz";
    }
    virtual bool Parse(const std::vector<std::string>& args);

    virtual void PrintHelp() {
        printf("WaitForAltitudeCommand z dz dvz - wait for drone to get with in dz of the specified local z, and settle down to given velocity delta (dvz).\n");
    }

    virtual void Execute(std::shared_ptr<MavLinkVehicle> com);
private:
    float z, dz, dvz;

};

#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <utility>
#include "common/Common.hpp"
#include "common/common_utils/Utils.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/AsyncTasker.hpp"
#include "rpc/RpcLibClient.hpp"
#include "SimpleShell.hpp"
#include "common/EarthUtils.hpp"
#include "controllers/DroneCommon.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "safety/SafetyEval.hpp"


namespace msr { namespace airlib {

using namespace std;
using namespace common_utils;

struct CommandContext {
public:
    RpcLibClient client;
    AsyncTasker tasker;
};

using DroneCommandParameters = SimpleShell<CommandContext>::ShellCommandParameters;
using DroneCommandSwitch = SimpleShell<CommandContext>::ShellCommandSwitch;

class DroneCommand : public SimpleShell<CommandContext>::ShellCommand {
public:
    DroneCommand(std::string name, std::string help)
        : ShellCommand(name,help)
    {
    }

    YawMode getYawMode()
    {
        std::string yaw = getSwitch("-yaw").value;
        std::string rate = getSwitch("-yaw_rate").value;
        if (yaw.empty())
        {
            if (rate.empty()) {
                return YawMode(false, 0);
            }
            return YawMode(true, std::stof(rate));
        }
        return YawMode(/*bool is_rate_val, float yaw_or_rate_val*/ false, std::stof(yaw));
    }

    void addYawModeSwitches(){
        this->addSwitch({"-yaw", "", "specify yaw in degrees (default not set)"});
        this->addSwitch({"-yaw_rate", "", "specify yaw rate in degrees per second (default not set)"});
    }

    void addLookaheadSwitches() {
        this->addSwitch({"-lookahead", "-1", "How far to look ahead on the path (default -1 means auto)" });
        this->addSwitch({"-adaptive_lookahead", "1", "Whether to apply adaptive lookahead (1=yes, 0=no) (default 1)" });
    }

    DrivetrainType getDriveTrain() {            
        int drivetrain = getSwitchInt("-drivetrain");
        return static_cast<DrivetrainType>(drivetrain);
    }

    void addDriveTrainSwitch() {
        this->addSwitch({"-drivetrain", "1", "type of drive mode (1=forward only, 0= max degree of freedom) (default is 1)" });
    }

};

class ArmCommand : public DroneCommand {
public:
    ArmCommand() : DroneCommand("Arm", "Arm the motors so the drone is ready to fly")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.armDisarm(true);
        return false;
    }
};

class DisarmCommand : public DroneCommand {
public:
    DisarmCommand() : DroneCommand("Disarm", "Disarm the motors so we can safly approach the drone")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.armDisarm(false);
        return false;
    }
};

class RequestControlCommand : public DroneCommand {
public:
    RequestControlCommand() : DroneCommand("RequestControl", "Take offboard control of drone")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.setOffboardMode(true);
        return false;
    }
};

class ReleaseControlCommand : public DroneCommand {
public:
    ReleaseControlCommand() : DroneCommand("ReleaseControl", "Release offboard control of drone")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.setOffboardMode(false);
        return false;
    }
};

class TakeOffCommand : public DroneCommand {
public:
    TakeOffCommand() : DroneCommand("TakeOff", "Drone takeoff to a default altitude")
    {
        this->addSwitch({"-takeoff_wait", "15", "specify time to wait after issuing command (default 15 seconds)" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.takeoff(std::stof(getSwitch("-takeoff_wait").value));
        return false;
    }
};


class LandCommand : public DroneCommand {
public:
    LandCommand() : DroneCommand("Land", "Land the drone")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.land();
        return false;
    }
};

class GoHomeCommand : public DroneCommand {
public:
    GoHomeCommand() : DroneCommand("GoHome", "Go back to takeoff point and land")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        params.context->client.goHome();
        return false;
    }
};

class GetHomePointCommand : public DroneCommand {
public:
    GetHomePointCommand() : DroneCommand("GetHomePoint", "Display the homepoint set in the drone")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        auto homepoint = params.context->client.getHomePoint();
        if (std::isnan(homepoint.longitude))
            params.shell_ptr->showMessage("Home point is not set!");
        else
            params.shell_ptr->showMessage(homepoint.to_string().c_str());

        return false;
    }
};


//move commands

class MoveToZCommand : public DroneCommand {
public:
    MoveToZCommand() : DroneCommand("MoveToZ", "Move to z in meters (measured from launch point) with given velocity in m/s")
    {
        this->addSwitch({"-z", "-2.5", "Move to specified z above launch position (default -2.5 meters)" });
        this->addSwitch({"-velocity", "0.5", "Velocity to move at (default 0.5 meters per second)" });
        addYawModeSwitches();
        addLookaheadSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float z = std::stof(getSwitch("-z").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveToZ(z, velocity, getYawMode(), lookahead, adaptive_lookahead);
        });

        return false;
    }
};

class RotateByYawRateCommand : public DroneCommand {
public:
    RotateByYawRateCommand() : DroneCommand("Rotate", "Rotate with angular velocity in degrees/s for given time in seconds")
    {
        this->addSwitch({"-duration", "5", "maximum time to wait (default 5 seconds)" });
        this->addSwitch({"-yaw_rate", "20", "degrees per second (default 20 degrees per second))" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float yaw_rate = std::stof(getSwitch("-yaw_rate").value);
        float duration = std::stof(getSwitch("-duration").value);
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.rotateByYawRate(yaw_rate, duration);
        });

        return false;
    }
};

class RotateToYawCommand : public DroneCommand {
public:
    RotateToYawCommand() : DroneCommand("RotateTo", "Rotate to a particular angle")
    {
        this->addSwitch({"-yaw_margin", "5", "how accurate to be (default within 5 degrees)" });
        this->addSwitch({"-yaw", "0", "degrees (default 0)" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float yaw = std::stof(getSwitch("-yaw").value);
        float margin = std::stof(getSwitch("-yaw_margin").value);
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.rotateToYaw(yaw, margin);
        });

        return false;
    }
};

class HoverCommand : public DroneCommand {
public:
    HoverCommand() : DroneCommand("Hover", "Enter hover mode")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        CommandContext* context = params.context;
        context->tasker.execute([=]() {
            context->client.hover();
        });

        return false;
    }
};

class MoveToPositionCommand : public DroneCommand {
public:
    MoveToPositionCommand() : DroneCommand("MoveToPosition", "Move to x,y,z with specified velocity")
    {
        this->addSwitch({"-x", "0", "x position in meters (default 0)" });
        this->addSwitch({"-y", "0", "y position in meters (default 0)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-velocity", "2.5", "the velocity to approach the position in meters per second (default 2.5)" });
        addYawModeSwitches();
        addDriveTrainSwitch();
        addLookaheadSwitches();
    }

    void MoveToPosition(CommandContext* context, float x, float y, float z, float velocity, DrivetrainType drivetrain, YawMode yawMode, float lookahead, float adaptive_lookahead)
    {
        context->client.moveToPosition(x, y, z, velocity,
            drivetrain, yawMode, lookahead, adaptive_lookahead);
    }

    bool execute(const DroneCommandParameters& params)
    {
        float x = std::stof(getSwitch("-x").value);
        float y = std::stof(getSwitch("-y").value);
        float z = std::stof(getSwitch("-z").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        auto drivetrain = getDriveTrain();
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            MoveToPosition(context, x, y, z, velocity,
                drivetrain, yawMode, lookahead, adaptive_lookahead);
        });

        return false;
    }
};

class MoveByManualCommand : public DroneCommand {
public:
    MoveByManualCommand() : DroneCommand("MoveByManual", "Move using remote control manually")
    {
        this->addSwitch({"-vx", "0", "velocity in x direction in meters per second (default 0)" });
        this->addSwitch({"-vy", "0", "velocity in y direction in meters per second (default 0)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });

        addDriveTrainSwitch();
        addLookaheadSwitches();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float vx = std::stof(getSwitch("-vx").value);
        float vy = std::stof(getSwitch("-vy").value);
        float z = std::stof(getSwitch("-z").value);
        float duration = std::stof(getSwitch("-duration").value);
        auto drivetrain = getDriveTrain();
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveByManual(vx, vy, z, drivetrain, yawMode, duration);
        });

        return false;
    }
};


class MoveByAngleCommand : public DroneCommand {
public:
    MoveByAngleCommand() : DroneCommand("MoveByAngle", "Move with specified roll and pitch, leaving z as-is")
    {
        this->addSwitch({"-pitch", "0", "pitch angle in degrees (default 0)" });
        this->addSwitch({"-roll", "0", "roll angle in degrees (default 0)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-yaw", "0", "target yaw angle in degrees (default is 0)" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float pitch = std::stof(getSwitch("-pitch").value);
        float roll = std::stof(getSwitch("-roll").value);
        float z = std::stof(getSwitch("-z").value);
        float yaw = std::stof(getSwitch("-yaw").value);
        float duration = std::stof(getSwitch("-duration").value);
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveByAngle(pitch, roll, z, yaw, duration);
        });

        return false;
    }
};

class MoveByVelocityCommand : public DroneCommand {
public:
    MoveByVelocityCommand() : DroneCommand("MoveByVelocity", "Move by specified velocity components vx, vy, vz, axis wrt body")
    {
        this->addSwitch({"-vx", "0", "velocity in x direction in meters per second (default 0)" });
        this->addSwitch({"-vy", "0", "velocity in y direction in meters per second (default 0)" });
        this->addSwitch({"-vz", "0", "velocity in z direction in meters per second (default 0)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        addDriveTrainSwitch();
        addYawModeSwitches();
    }


    bool execute(const DroneCommandParameters& params) 
    {
        float vx = std::stof(getSwitch("-vx").value);
        float vy = std::stof(getSwitch("-vy").value);
        float vz = std::stof(getSwitch("-vz").value);
        float duration = std::stof(getSwitch("-duration").value);
        auto drivetrain = getDriveTrain();
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveByVelocity(vx, vy, vz, duration, drivetrain, yawMode);
        });

        return false;
    }
};

class MoveByVelocityZCommand : public DroneCommand {
public:
    MoveByVelocityZCommand() : DroneCommand("MoveByVelocityZ", "Move by specified velocity components vx, vy, and fixed z")
    {
        this->addSwitch({"-vx", "0", "velocity in x direction in meters per second (default 0)" });
        this->addSwitch({"-vy", "0", "velocity in y direction in meters per second (default 0)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        addDriveTrainSwitch();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float vx = std::stof(getSwitch("-vx").value);
        float vy = std::stof(getSwitch("-vy").value);
        float z = std::stof(getSwitch("-z").value);
        float duration = std::stof(getSwitch("-duration").value);
        auto drivetrain = getDriveTrain();
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveByVelocityZ(vx, vy, z, duration, drivetrain, yawMode);
        });

        return false;
    }
};

class SetSafetyCommand : public DroneCommand {
public:
    SetSafetyCommand() : DroneCommand("SetSafety", "Set safety parameters")
    {
        this->addSwitch({"-safety_flags", "1", "0 = none, -1 = all, 1 = geofence, 2 = obstacle (default is 1)" });
        this->addSwitch({"-obs_clearance", "", "safe distance from obstacles (no default)" });
        this->addSwitch({"-obs_avoidance_vel", "0.5", "velocity to move away from obstacles (default 0.5)" });
        this->addSwitch({"-obs_strategy", "0", "0 = none/exception, 1 = closest move, 2 = opposite move (default 0)" });
        this->addSwitch({"-xy_length", "", "geofence boundary size (no default)" });
        this->addSwitch({"-max_z", "", "geofence min altitude (no default)" });
        this->addSwitch({"-min_z", "", "geofence max altitude (no default)" });
        this->addSwitch({"-origin", "", "geofence origin as 'x,y,z' local position in meters (no default)" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        int safety_flags_switch = std::stoi(getSwitch("-safety_flags").value);
        float obs_clearance = std::stof(getSwitch("-obs_clearance").value);
        float obs_avoidance_vel = std::stof(getSwitch("-obs_avoidance_vel").value);
        uint obs_strategy = std::stoi(getSwitch("-obs_strategy").value);
        CommandContext* context = params.context;

        uint safety_flags = static_cast<uint>(safety_flags_switch);
        if (safety_flags_switch == -1)
            safety_flags = Utils::max<unsigned int>();

        Vector3r origin(Utils::nan<float>(), Utils::nan<float>(), Utils::nan<float>());
        float max_z = Utils::nan<float>(), min_z = Utils::nan<float>(), xy_length = Utils::nan<float>();

        std::string s_xy_length = getSwitch("-xy_length").value;
        if (!s_xy_length.empty()){
            xy_length = std::stof(s_xy_length);
        }

        std::string s_min_z = getSwitch("-min_z").value;
        if (!s_min_z.empty()){
            min_z = std::stof(s_min_z);
        }

        std::string s_max_z = getSwitch("-max_z").value;
        if (!s_max_z.empty()){
            max_z = std::stof(s_max_z);
        }

        std::string s_origin = getSwitch("-origin").value;
        if (!s_origin.empty()){
            std::vector<std::string> parts;
            boost::algorithm::split(parts, s_origin, boost::is_any_of(","));
            if (parts.size() == 3){            
                origin[0] = std::stof(parts[0]); 
                origin[1] = std::stof(parts[1]); 
                origin[2] = std::stof(parts[2]);
            } else {
                throw std::invalid_argument("-origin argument is expectin 'x,y,z' (separated by commas and no spaces in between)");
            }
        }

        context->tasker.execute([=]() {
            context->client.setSafety(SafetyEval::SafetyViolationType(safety_flags), obs_clearance,
                SafetyEval::ObsAvoidanceStrategy(obs_strategy), obs_avoidance_vel, origin, xy_length, min_z, max_z);
        });

        return false;
    }
};


class BackForthByAngleCommand : public DroneCommand {
public:
    BackForthByAngleCommand() : DroneCommand("BackForthByAngle", "Make drone go in linear motion back and forth using pitch/roll")
    {
        this->addSwitch({"-pitch", "0", "pitch angle in degrees (default 0)" });
        this->addSwitch({"-roll", "0", "roll angle in degrees (default 0)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-yaw", "0", "target yaw angle in degrees (default is 0)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float pitch = std::stof(getSwitch("-pitch").value);
        float roll = std::stof(getSwitch("-roll").value);
        float z = std::stof(getSwitch("-z").value);
        float duration = std::stof(getSwitch("-duration").value);
        float yaw = std::stof(getSwitch("-yaw").value);
        float pause_time = std::stof(getSwitch("-pause_time").value);
        int iterations = getSwitchInt("-iterations");
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveByAngle(pitch, roll, z, yaw, duration);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveByAngle(-pitch, -roll, z, yaw, duration);
        }, iterations);
        
        return false;
    }
};

class BackForthByPositionCommand : public DroneCommand {
public:
    BackForthByPositionCommand() : DroneCommand("BackForthByPosition", "Make drone go in linear motion back and forth two x positions")
    {
        this->addSwitch({"-length", "2.5", "length of each run on the x-axis (default 2.5)" });
        this->addSwitch({"-velocity", "0.5", "velocity in meters per second (default 0.5)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });
        addLookaheadSwitches();
        addDriveTrainSwitch();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float length = std::stof(getSwitch("-length").value);
        float z = std::stof(getSwitch("-z").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        float pause_time = std::stof(getSwitch("-pause_time").value);
        int iterations = getSwitchInt("-iterations");
        auto drivetrain = getDriveTrain();
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveToPosition(length, 0, z, velocity, drivetrain,
                yawMode, lookahead, adaptive_lookahead);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveToPosition(-length, 0, z, velocity, drivetrain,
                yawMode, lookahead, adaptive_lookahead);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
        }, iterations);

        return false;
    }
};


class SquareByAngleCommand : public DroneCommand {
public:
    SquareByAngleCommand() : DroneCommand("SquareByAngle", "Make drone go in square using pitch/roll")
    {
        this->addSwitch({"-pitch", "0", "pitch angle in degrees (default 0)" });
        this->addSwitch({"-roll", "0", "roll angle in degrees (default 0)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-yaw", "0", "target yaw angle in degrees (default is 0)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float pitch = std::stof(getSwitch("-pitch").value);
        float roll = std::stof(getSwitch("-roll").value);
        float z = std::stof(getSwitch("-z").value);
        float duration = std::stof(getSwitch("-duration").value);
        float yaw = std::stof(getSwitch("-yaw").value);
        float pause_time = std::stof(getSwitch("-pause_time").value);
        int iterations = getSwitchInt("-iterations");
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveByAngle(pitch, -roll, z, yaw, duration);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveByAngle(-pitch, -roll, z, yaw, duration);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveByAngle(-pitch, roll, z, yaw, duration);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveByAngle(-pitch, -roll, z, yaw, duration);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
        }, iterations);

        return false;
    }
};

class SquareByPositionCommand : public DroneCommand {
public:
    SquareByPositionCommand() : DroneCommand("SquareByPosition", "Make drone go in square using position commands")
    {
        this->addSwitch({"-length", "2.5", "length of each side (default 2.5)" });
        this->addSwitch({"-velocity", "0.5", "velocity in meters per second (default 0.5)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });
        addLookaheadSwitches();
        addDriveTrainSwitch();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float length = std::stof(getSwitch("-length").value);
        float z = std::stof(getSwitch("-z").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        float pause_time = std::stof(getSwitch("-pause_time").value);
        int iterations = getSwitchInt("-iterations");
        auto drivetrain = getDriveTrain();
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            context->client.moveToPosition(length, -length, z, velocity, drivetrain,
                yawMode, lookahead, adaptive_lookahead);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveToPosition(-length, -length, z, velocity, drivetrain,
                yawMode, lookahead, adaptive_lookahead);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveToPosition(-length, length, z, velocity, drivetrain,
                yawMode, lookahead, adaptive_lookahead);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            context->client.moveToPosition(length, length, z, velocity, drivetrain,
                yawMode, lookahead, adaptive_lookahead);
            context->client.hover();
            std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
        }, iterations);

        return false;
    }
};

class SquareByPathCommand : public DroneCommand {
public:
    SquareByPathCommand() : DroneCommand("SquareByPath", "Make drone go in square using path commands")
    {
        this->addSwitch({"-length", "2.5", "length of each side (default 2.5)" });
        this->addSwitch({"-velocity", "0.5", "velocity in meters per second (default 0.5)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });
        this->addSwitch({"-path_rep", "1000", "number of times around the square (default 1000)" });
        addLookaheadSwitches();
        addDriveTrainSwitch();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float length = std::stof(getSwitch("-length").value);
        float z = std::stof(getSwitch("-z").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        //float pause_time = std::stof(getSwitch("-pause_time").value);
        int iterations = getSwitchInt("-iterations");
        auto drivetrain = getDriveTrain();
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        int path_rep = std::stoi(getSwitch("-path_rep").value);
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        std::vector<Vector3r> path;
        for(int i = 0; i < path_rep; ++i) {
            path.push_back(Vector3r(length, -length, z));
            path.push_back(Vector3r(-length, -length, z));
            path.push_back(Vector3r(-length, length, z));
            path.push_back(Vector3r(length, length, z));
        }

        context->tasker.execute([=]() {
            context->client.moveOnPath(path, velocity, drivetrain, yawMode, lookahead, adaptive_lookahead);
        }, iterations);

        return false;
    }
};


class CircleByPositionCommand : public DroneCommand {
public:
    CircleByPositionCommand() : DroneCommand("CircleByPosition", "Make drone go in square using position commands")
    {
        this->addSwitch({"-radius", "2.5", "radius of circle (default 2.5)" });
        this->addSwitch({"-velocity", "0.5", "velocity in meters per second (default 0.5)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-seg_length", "0.2", "circle is approximated using strait segments (default length .2 meters)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });

        addLookaheadSwitches();
        addDriveTrainSwitch();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float radius = std::stof(getSwitch("-radius").value);
        float z = std::stof(getSwitch("-z").value);
        float seg_length = std::stof(getSwitch("-seg_length").value);
        float pause_time = std::stof(getSwitch("-pause_time").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        auto drivetrain = getDriveTrain();
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        int iterations = getSwitchInt("-iterations");
        auto yawMode = getYawMode();

        float angle = std::acos(1 - (seg_length*seg_length / (2 * radius*radius)));
        if (std::isnan(angle))
            throw std::invalid_argument(common_utils::Utils::stringf("radius=%f and seg_length=%f doesn't form valid circle", radius, seg_length));
        int seg_count = common_utils::Utils::floorToInt(2*M_PIf / angle);
        float seg_angle = 2*M_PIf / seg_count;
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            for(float seg = 0; seg < seg_count; ++seg) {
                float x = std::cos(seg_angle * seg) * radius;
                float y = std::sin(seg_angle * seg) * radius;
                context->client.moveToPosition(x, y, z, velocity, drivetrain,
                    yawMode, lookahead, adaptive_lookahead);
                context->client.hover();
                std::this_thread::sleep_for(std::chrono::duration<double>(pause_time));
            }
        }, iterations);

        return false;
    }
};

class CircleByPathCommand : public DroneCommand {
public:
    CircleByPathCommand() : DroneCommand("CircleByPath", "Make drone go in circle using path commands")
    {
        this->addSwitch({"-radius", "2.5", "radius of circle (default 2.5)" });
        this->addSwitch({"-velocity", "0.5", "velocity in meters per second (default 0.5)" });
        this->addSwitch({"-z", "-2.5", "z position in meters (default -2.5)" });
        this->addSwitch({"-duration", "5", "the duration of this command in seconds (default 5)" });
        this->addSwitch({"-pause_time", "0", "pause time between each run back and forth in seconds (default 0)" });
        this->addSwitch({"-iterations", "10000", "number of times to repeat the task (default 10000)" });
        this->addSwitch({"-path_rep", "1000", "number of times around the square (default 1000)" });
        this->addSwitch({"-plane", "xy", "which plane to fly (default xy)" });
        this->addSwitch({"-seg_length", "0.2", "circle is approximated using strait segments (default length .2 meters)" });
        addLookaheadSwitches();
        addDriveTrainSwitch();
        addYawModeSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        using std::swap;

        float radius = std::stof(getSwitch("-radius").value);
        float z_path = std::stof(getSwitch("-z").value);
        float seg_length = std::stof(getSwitch("-seg_length").value);
        float velocity = std::stof(getSwitch("-velocity").value);
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        auto drivetrain = getDriveTrain();
        int iterations = getSwitchInt("-iterations");
        int path_rep = std::stoi(getSwitch("-path_rep").value);
        string plane = getSwitch("-plane").value;
        auto yawMode = getYawMode();
        CommandContext* context = params.context;

        float angle = std::acos(1 - (seg_length*seg_length / (2 * radius*radius)));
        if (std::isnan(angle))
            throw std::invalid_argument(common_utils::Utils::stringf("radius=%f and seg_length=%f doesn't form valid circle", radius, seg_length));
        int seg_count = common_utils::Utils::floorToInt(2*M_PIf / angle);
        float seg_angle = 2*M_PIf / seg_count;

        const Vector3r origin = plane != "xy" && plane != "yx" ? Vector3r(0, 0, z_path + radius) : Vector3r(0, 0, z_path);

        std::vector<Vector3r> path;
        for(int i = 0; i < path_rep; ++i) {
            for(float seg = 0; seg < seg_count; ++seg) {
                float x = std::cos(seg_angle * seg) * radius;
                float y = std::sin(seg_angle * seg) * radius;
                float z = 0;

                if (plane == "yx") swap(x, y);
                else if (plane == "xz") swap(y, z);
                else if (plane == "zx") { swap(y, z); swap(z, x); }
                else if (plane == "yz") swap(x, z);
                else if (plane == "zy") { swap(x, z); swap(z, y); }
                //else leave as it is

                path.push_back(Vector3r(x, y, z) + origin);
            }
        }

        context->tasker.execute([=]() {
            context->client.moveOnPath(path, velocity, drivetrain, yawMode, lookahead, adaptive_lookahead);
        }, iterations);

        return false;
    }
};

class RecordPoseCommand : public DroneCommand {
public:
    RecordPoseCommand() : DroneCommand("CircleByPath", "Append a single pose snapshot to a log file named 'rec_pos.log' in your $HOME folder\n\
Each record is tab separated floating point numbers containing GPS lat,lon,alt,z,health, position x,y,z, and quaternion w,x,y,z")
    {
    }

    bool execute(const DroneCommandParameters& params) 
    {
        //TODO: get these in one call
        Vector3r position = params.context->client.getPosition();
        Quaternionr quaternion = params.context->client.getOrientation();
        GeoPoint gps_point = params.context->client.getGpsLocation();

        params.shell_ptr->showMessage(gps_point.to_string());
        params.shell_ptr->showMessage(VectorMath::toString(position));
        params.shell_ptr->showMessage(VectorMath::toString(quaternion));

        string line = Utils::stringf("%f\t%f\t%f\t%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
            gps_point.latitude, gps_point.longitude, gps_point.altitude, //TODO: gps_point.height, gps_point.health,
            position[0], position[1], position[2], 
            quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()
        );

        std::string file_path_name = FileSystem::getLogFileNamePath("rec_pos", "", ".log", false);
        FileSystem::appendLineToFile(file_path_name, line);

        return false;
    }
};


//shell.addCommand("PlayPose", &playPoseCommand, "Play position, quaternion and GPS coordinates of drone from log file");

class PlayPoseCommand : public DroneCommand {
public:
    PlayPoseCommand() : DroneCommand("PlayPose", "Fly the drone through each recorded pose found in log file named 'rec_pos.log' in your $HOME folder\n\
See RecordPose for information about log file format")
    {
        this->addSwitch({"-velocity", "0.5", "Velocity to move at (default 0.5 meters per second)" });
        addLookaheadSwitches();
    }

    bool execute(const DroneCommandParameters& params) 
    {
        float velocity = std::stof(getSwitch("-velocity").value);
        float lookahead = std::stof(getSwitch("-lookahead").value);
        float adaptive_lookahead = std::stof(getSwitch("-adaptive_lookahead").value);
        CommandContext* context = params.context;

        context->tasker.execute([=]() {
            std::ifstream file;
            std::string file_path_name = FileSystem::getLogFileNamePath("rec_pos", "", ".log", false);
            file.exceptions(file.exceptions() | std::ios::failbit);
            FileSystem::openTextFile(file_path_name, file);

            Vector3r position;
            Quaternionr quaternion;
            GeoPoint gps_point;
            float pitch, roll, yaw;

            while (!file.eof()) {
                string line = FileSystem::readLineFromFile(file);
                if (line.length() > 0) {
                    //parse
                    std::stringstream line_ss(line);
                    line_ss >> gps_point.latitude >> gps_point.longitude >> gps_point.altitude; // >> gps_point.height >> gps_point.health;
                    line_ss >> position[0] >> position[1] >> position[2];
                    line_ss >> quaternion.w() >> quaternion.x() >> quaternion.y() >> quaternion.z();

                    params.shell_ptr->showMessage(gps_point.to_string());
                    params.shell_ptr->showMessage(VectorMath::toString(position));
                    params.shell_ptr->showMessage(VectorMath::toString(quaternion));

                    GeoPoint home_point = context->client.getHomePoint();
                    Vector3r local_point = EarthUtils::GeodeticToNedFast(gps_point, home_point);
                    VectorMath::toEulerianAngle(quaternion, pitch, roll, yaw);

                    context->client.moveToPosition(local_point.x(), local_point.y(), local_point.z(), velocity, 
                        DrivetrainType::MaxDegreeOfFreedome, YawMode(false, yaw), lookahead, adaptive_lookahead);
                }
            }
        });

        return false;
    }
};


class GetImageCommand : public DroneCommand {
public:
    GetImageCommand() : DroneCommand("GetImage", "Get an image from the simulator")
    {
        this->addSwitch({"-type", "depth", "scene, depth, or segmentation" });
        this->addSwitch({"-name", "image", "name of the file" });
    }

    bool execute(const DroneCommandParameters& params) 
    {
        std::string type = getSwitch("-type").value;
        std::string name = getSwitch("-name").value;
        CommandContext* context = params.context;

        DroneControllerBase::ImageType imageType;

        if (type == "depth") {
            imageType = DroneControllerBase::ImageType::Depth;
        } else if (type == "scene") {
            imageType = DroneControllerBase::ImageType::Scene;
        } else if (type == "segmentation") {
            imageType = DroneControllerBase::ImageType::Segmentation;
        } else {
            cout << "Error: Invalid image type '" << type << "', expecting either 'depth', 'scene' or 'segmentation'" << endl;
            return true;
        }

        context->tasker.execute([=]() {
            std::string file_path_name = FileSystem::getLogFileNamePath(name, "", ".png", false);

            context->client.setImageTypeForCamera(0, imageType);

            auto image = context->client.getImageForCamera(0, imageType);

            // if we are too quick we miss the image.
            while (image.size() <= 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                image = context->client.getImageForCamera(0, imageType);
            }
            ofstream file;
            FileSystem::createBinaryFile(file_path_name, file);
            file.write((char*) image.data(), image.size());
            file.close();

            cout << "Image saved to: " << name << " (" << image.size() << " bytes)" << endl;
            
        });

        return false;
    }
};


// std::string beforeScriptStartCallback(const DroneCommandParameters& params, std::string scriptFilePath) 
// {
//     return "";
// }
// bool afterScriptEndCallback(const DroneCommandParameters& params, std::string scriptFilePath) 
// {
//     return false;
// }
// std::string beforeScriptCommandStartCallback(const DroneCommandParameters& params) {
//     params.context->client.newTask();
// }
// bool afterScriptCommandEndCallback(const DroneCommandParameters& params, bool commandReturnValue) {
//     params.context->client.WaitForCompletion(0);
// }
//void beforeCommandStartCallback(const DroneCommandParameters& params, std::string command_line) 
//{
//}


}} //namespace


std::string server_address("127.0.0.1");

bool parseCommandLine(int argc, const char* argv[])
{
    // parse command line
    for (int i = 1; i < argc; i++) {
        const char* arg = argv[i];
        if (arg[0] == '-' || arg[0] == '/') {
            std::string name = arg + 1;
            if (name == "server" && i + 1 < argc) {
                server_address = argv[++i];
            } else {
                return false;
            }
        }
        else {
            return false;
        }
    }
    return true;
}

void printUsage() {
    std::cout << "Usage: DroneServer [-server 127.0.0.1]" << std::endl;
    std::cout << "The default server address is 127.0.0.1, but use the -server option to specify a different address for the server" << std::endl;
}

int main(int argc, const char *argv[]) {

    using namespace msr::airlib;
    

    if (!parseCommandLine(argc, argv)) {
        printUsage();
        return -1;
    }

    CommandContext command_context{ /*RpcClient*/{server_address}, /*AsyncTasker*/ {} };

    command_context.tasker.setErrorHandler([](std::exception& e) {
        try {
            rpc::rpc_error& rpc_ex = dynamic_cast<rpc::rpc_error&>(e);
            std::cerr << "Async RPC Error: " << rpc_ex.get_error().as<std::string>() << std::endl;
        } 
        catch (...) {
            std::cerr << "Async Error occurred: " << e.what() << std::endl;
        }
    });

    SimpleShell<CommandContext> shell("==||=> ");

    shell.showMessage(R"(
        Welcome to DroneShell 1.0.
        Type ? for help.
        Microsoft Research (c) 2016.
    )");

    //Shell callbacks
    // shell.beforeScriptStartCallback(std::bind(&beforeScriptStartCallback, std::placeholders::_1, std::placeholders::_2));
    // shell.afterScriptEndCallback(std::bind(&afterScriptEndCallback, std::placeholders::_1, std::placeholders::_2));
    // shell.afterScriptCommandEndCallback(std::bind(&afterScriptCommandEndCallback, std::placeholders::_1, std::placeholders::_2));
    //shell.beforeCommandStartCallback(std::bind(&beforeCommandStartCallback, std::placeholders::_1, std::placeholders::_2));

    //Add shell commands
    ArmCommand arm;
    DisarmCommand disarm;
    RequestControlCommand requestControl;
    ReleaseControlCommand releaseControl;
    TakeOffCommand takeOff;
    LandCommand land;
    GoHomeCommand goHome;
    //TODO: add WaitForCompletion command
    GetHomePointCommand getHomePoint;
    MoveToZCommand moveToZ;
    RotateByYawRateCommand rotateByYawRate;
    RotateToYawCommand rotateToYaw;
    HoverCommand hover;
    MoveToPositionCommand moveToPosition;
    MoveByManualCommand moveByManual;
    MoveByAngleCommand moveByAngle;
    MoveByVelocityCommand moveByVelocity;
    MoveByVelocityZCommand moveByVelocityZ;
    SetSafetyCommand setSafety;
    BackForthByAngleCommand backForthByAngle;
    BackForthByPositionCommand backForthByPosition;
    SquareByAngleCommand squareByAngle;
    SquareByPositionCommand squareByPosition;
    SquareByPathCommand squareByPath;
    CircleByPositionCommand circleByPosition;
    CircleByPathCommand circleByPath;
    RecordPoseCommand  recordPose;
    PlayPoseCommand playPose;
    GetImageCommand imageCommand;

    //TODO: add command line args help, arg count validation
    shell.addCommand(arm);
    shell.addCommand(disarm);
    shell.addCommand(requestControl);
    shell.addCommand(releaseControl);
    shell.addCommand(takeOff);
    shell.addCommand(land);
    shell.addCommand(goHome);
    shell.addCommand(getHomePoint);
    shell.addCommand(moveToZ);
    shell.addCommand(rotateByYawRate);
    shell.addCommand(rotateToYaw);
    shell.addCommand(hover);
    shell.addCommand(moveToPosition);
    shell.addCommand(moveByManual);
    shell.addCommand(moveByAngle);
    shell.addCommand(moveByVelocity);
    shell.addCommand(moveByVelocityZ);
    shell.addCommand(setSafety);
    shell.addCommand(backForthByAngle);
    shell.addCommand(backForthByPosition);
    shell.addCommand(squareByAngle);
    shell.addCommand(squareByPosition);
    shell.addCommand(squareByPath);
    shell.addCommand(circleByPosition);
    shell.addCommand(circleByPath);
    shell.addCommand(recordPose);
    shell.addCommand(playPose);
    shell.addCommand(imageCommand);

    while(!shell.readLineAndExecute(&command_context)) {
    }

    return 0;
}

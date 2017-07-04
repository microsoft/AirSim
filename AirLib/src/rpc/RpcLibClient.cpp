// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif


#include "common/Common.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#include <rpc/client.h>
#include "rpc/RpcLibAdapators.hpp"
STRICT_MODE_ON
#include "rpc/RpcLibClient.hpp"
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {

struct RpcLibClient::impl {
    impl(const string&  ip_address, uint16_t port)
        : client(ip_address, port)
    {
        // some long flight path commands can take a while, so we give it up to 1 hour max.
        client.set_timeout(3600*1000);
    }

    rpc::client client;
};

typedef msr::airlib_rpclib::RpcLibAdapators RpcLibAdapators;

RpcLibClient::RpcLibClient(const string&  ip_address, uint16_t port)
{
    pimpl_.reset(new impl(ip_address, port));
}

RpcLibClient::~RpcLibClient()
{}

bool RpcLibClient::ping()
{
    return pimpl_->client.call("ping").as<bool>();
}
RpcLibClient::ConnectionState RpcLibClient::getConnectionState()
{
    switch (pimpl_->client.get_connection_state()) {
    case rpc::client::connection_state::connected: return ConnectionState::Connected;
    case rpc::client::connection_state::disconnected: return ConnectionState::Disconnected;
    case rpc::client::connection_state::initial: return ConnectionState::Initial;
    case rpc::client::connection_state::reset: return ConnectionState::Reset;
    default:
        return ConnectionState::Unknown;
    }
}
bool RpcLibClient::armDisarm(bool arm)
{
    return pimpl_->client.call("armDisarm", arm).as<bool>();
}
void RpcLibClient::setOffboardMode(bool is_set)
{
    pimpl_->client.call("setOffboardMode", is_set);
}
void RpcLibClient::setSimulationMode(bool is_set)
{
    pimpl_->client.call("setSimulationMode", is_set);
}
bool RpcLibClient::takeoff(float max_wait_seconds)
{
    return pimpl_->client.call("takeoff", max_wait_seconds).as<bool>();
}
bool RpcLibClient::land(float max_wait_seconds)
{
    return pimpl_->client.call("land", max_wait_seconds).as<bool>();
}
bool RpcLibClient::goHome()
{
    return pimpl_->client.call("goHome").as<bool>();
}
void RpcLibClient::start()
{
    pimpl_->client.call("start");
}
void RpcLibClient::stop()
{
    pimpl_->client.call("stop");
}


bool RpcLibClient::moveByAngle(float pitch, float roll, float z, float yaw, float duration)
{
    return pimpl_->client.call("moveByAngle", pitch, roll, z, yaw, duration).as<bool>();
}

bool RpcLibClient::moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return pimpl_->client.call("moveByVelocity", vx, vy, vz, duration, drivetrain, RpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool RpcLibClient::moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return pimpl_->client.call("moveByVelocityZ", vx, vy, z, duration, drivetrain, RpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool RpcLibClient::moveOnPath(const vector<Vector3r>& path, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    vector<RpcLibAdapators::Vector3r> conv_path;
    RpcLibAdapators::from(path, conv_path);
    return pimpl_->client.call("moveOnPath", conv_path, velocity, max_wait_seconds, drivetrain, RpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool RpcLibClient::moveToPosition(float x, float y, float z, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    return pimpl_->client.call("moveToPosition", x, y, z, velocity, max_wait_seconds, drivetrain, RpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool RpcLibClient::moveToZ(float z, float velocity, float max_wait_seconds, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    return pimpl_->client.call("moveToZ", z, velocity, max_wait_seconds, RpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool RpcLibClient::moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return pimpl_->client.call("moveByManual", vx_max, vy_max, z_min, duration, drivetrain, RpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool RpcLibClient::rotateToYaw(float yaw, float max_wait_seconds, float margin)
{
    return pimpl_->client.call("rotateToYaw", yaw, max_wait_seconds, margin).as<bool>();
}

bool RpcLibClient::rotateByYawRate(float yaw_rate, float duration)
{
    return pimpl_->client.call("rotateByYawRate", yaw_rate, duration).as<bool>();
}

bool RpcLibClient::hover()
{
    return pimpl_->client.call("hover").as<bool>();
}

bool RpcLibClient::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
    float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
{
    return pimpl_->client.call("setSafety", static_cast<uint>(enable_reasons), obs_clearance, obs_startegy,
        obs_avoidance_vel, RpcLibAdapators::Vector3r(origin), xy_length, max_z, min_z).as<bool>();
}

//status getters
Vector3r RpcLibClient::getPosition()
{
    return pimpl_->client.call("getPosition").as<RpcLibAdapators::Vector3r>().to();
}
Vector3r RpcLibClient::getVelocity()
{
    return pimpl_->client.call("getVelocity").as<RpcLibAdapators::Vector3r>().to();
}
Quaternionr RpcLibClient::getOrientation()
{
    return pimpl_->client.call("getOrientation").as<RpcLibAdapators::Quaternionr>().to();
}

DroneControllerBase::LandedState RpcLibClient::getLandedState()
{
    int result = pimpl_->client.call("getLandedState").as<int>();
    return static_cast<DroneControllerBase::LandedState>(result);
}

RCData RpcLibClient::getRCData()
{
    return pimpl_->client.call("getRCData").as<RpcLibAdapators::RCData>().to();
}

CollisionInfo RpcLibClient::getCollisionInfo()
{
    return pimpl_->client.call("getCollisionInfo").as<RpcLibAdapators::CollisionInfo>().to();
}

TTimePoint RpcLibClient::timestampNow()
{
    return pimpl_->client.call("timestampNow").as<TTimePoint>();
}

GeoPoint RpcLibClient::getHomePoint()
{
    return pimpl_->client.call("getHomePoint").as<RpcLibAdapators::GeoPoint>().to();
}

GeoPoint RpcLibClient::getGpsLocation()
{
    return pimpl_->client.call("getGpsLocation").as<RpcLibAdapators::GeoPoint>().to();
}

bool RpcLibClient::isOffboardMode()
{
    return pimpl_->client.call("isOffboardMode").as<bool>();
}

bool RpcLibClient::isSimulationMode()
{
    return pimpl_->client.call("isSimulationMode").as<bool>();
}

std::string RpcLibClient::getDebugInfo()
{
    return pimpl_->client.call("getDebugInfo").as<std::string>();
}

void RpcLibClient::confirmConnection()
{
    ClockBase* clock = ClockFactory::get();

    // make sure we can talk to the DroneServer
    //std::cout << "Contacting DroneServer..." << std::flush;
    //command_context.client.ping();
    //std::cout << "DroneServer is responding." << std::endl;

    std::cout << "Waiting for connection - " << std::flush;
    const TTimeDelta pause_time = 1;
    while (getConnectionState() != RpcLibClient::ConnectionState::Connected)
    {
        std::cout << "X" << std::flush;
        clock->sleep_for(pause_time); 
    }
    std::cout << "Connected!" << std::endl;

    std::cout << "Waiting for drone to report a valid GPS location..." << std::flush;
    auto gps = getGpsLocation();
    int count = 0;
    while (gps.latitude == 0 && gps.longitude == 0 && gps.altitude == 0 && count++ < 10)
    {
        std::cout << "." << std::flush;
        clock->sleep_for(pause_time); 
        gps = getGpsLocation();
    }
    std::cout << std::endl;
    std::cout << "Global position: lat=" << gps.latitude << ", lon=" << gps.longitude << ", alt=" << gps.altitude << std::endl;
}

//get/set image
vector<uint8_t> RpcLibClient::getImageForCamera(int camera_id, VehicleCamera::ImageType type)
{
    vector<uint8_t> result = pimpl_->client.call("getImageForCamera", camera_id, type).as<vector<uint8_t>>();
    if (result.size() == 1) {
        // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
        result.clear();
    }
    return result;
}


}} //namespace

#endif
#endif

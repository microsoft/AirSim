// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include "common/Common.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#include "rpc/client.h"
#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {

struct MultirotorRpcLibClient::impl {
    impl(const string&  ip_address, uint16_t port, uint timeout_ms)
        : client(ip_address, port)
    {
        // some long flight path commands can take a while, so we give it up to 1 hour max.
        client.set_timeout(timeout_ms);
    }

    rpc::client client;
};

typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

MultirotorRpcLibClient::MultirotorRpcLibClient(const string&  ip_address, uint16_t port, uint timeout_ms)
{
    pimpl_.reset(new impl(ip_address, port, timeout_ms));
}

MultirotorRpcLibClient::~MultirotorRpcLibClient()
{}

bool MultirotorRpcLibClient::ping()
{
    return pimpl_->client.call("ping").as<bool>();
}
MultirotorRpcLibClient::ConnectionState MultirotorRpcLibClient::getConnectionState()
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
bool MultirotorRpcLibClient::armDisarm(bool arm)
{
    return pimpl_->client.call("armDisarm", arm).as<bool>();
}
void MultirotorRpcLibClient::enableApiControl(bool is_enabled)
{
    pimpl_->client.call("enableApiControl", is_enabled);
}
void MultirotorRpcLibClient::setSimulationMode(bool is_set)
{
    pimpl_->client.call("setSimulationMode", is_set);
}
bool MultirotorRpcLibClient::takeoff(float max_wait_seconds)
{
    return pimpl_->client.call("takeoff", max_wait_seconds).as<bool>();
}
bool MultirotorRpcLibClient::land(float max_wait_seconds)
{
    return pimpl_->client.call("land", max_wait_seconds).as<bool>();
}
bool MultirotorRpcLibClient::goHome()
{
    return pimpl_->client.call("goHome").as<bool>();
}
void MultirotorRpcLibClient::start()
{
    pimpl_->client.call("start");
}
void MultirotorRpcLibClient::stop()
{
    pimpl_->client.call("stop");
}


bool MultirotorRpcLibClient::moveByAngle(float pitch, float roll, float z, float yaw, float duration)
{
    return pimpl_->client.call("moveByAngle", pitch, roll, z, yaw, duration).as<bool>();
}

bool MultirotorRpcLibClient::moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return pimpl_->client.call("moveByVelocity", vx, vy, vz, duration, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool MultirotorRpcLibClient::moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return pimpl_->client.call("moveByVelocityZ", vx, vy, z, duration, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool MultirotorRpcLibClient::moveOnPath(const vector<Vector3r>& path, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    vector<MultirotorRpcLibAdapators::Vector3r> conv_path;
    MultirotorRpcLibAdapators::from(path, conv_path);
    return pimpl_->client.call("moveOnPath", conv_path, velocity, max_wait_seconds, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool MultirotorRpcLibClient::moveToPosition(float x, float y, float z, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    return pimpl_->client.call("moveToPosition", x, y, z, velocity, max_wait_seconds, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool MultirotorRpcLibClient::moveToZ(float z, float velocity, float max_wait_seconds, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    return pimpl_->client.call("moveToZ", z, velocity, max_wait_seconds, MultirotorRpcLibAdapators::YawMode(yaw_mode), lookahead, adaptive_lookahead).as<bool>();
}

bool MultirotorRpcLibClient::moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    return pimpl_->client.call("moveByManual", vx_max, vy_max, z_min, duration, drivetrain, MultirotorRpcLibAdapators::YawMode(yaw_mode)).as<bool>();
}

bool MultirotorRpcLibClient::rotateToYaw(float yaw, float max_wait_seconds, float margin)
{
    return pimpl_->client.call("rotateToYaw", yaw, max_wait_seconds, margin).as<bool>();
}

bool MultirotorRpcLibClient::rotateByYawRate(float yaw_rate, float duration)
{
    return pimpl_->client.call("rotateByYawRate", yaw_rate, duration).as<bool>();
}

bool MultirotorRpcLibClient::hover()
{
    return pimpl_->client.call("hover").as<bool>();
}

bool MultirotorRpcLibClient::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
    float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
{
    return pimpl_->client.call("setSafety", static_cast<uint>(enable_reasons), obs_clearance, obs_startegy,
        obs_avoidance_vel, MultirotorRpcLibAdapators::Vector3r(origin), xy_length, max_z, min_z).as<bool>();
}

//sim only
void MultirotorRpcLibClient::simSetPose(const Pose& pose, bool ignore_collison)
{
    pimpl_->client.call("simSetPose", MultirotorRpcLibAdapators::Pose(pose), ignore_collison);
}
Pose MultirotorRpcLibClient::simGetPose()
{
    return pimpl_->client.call("simGetPose").as<MultirotorRpcLibAdapators::Pose>().to();
}
vector<VehicleCameraBase::ImageResponse> MultirotorRpcLibClient::simGetImages(vector<VehicleCameraBase::ImageRequest> request)
{
    const auto& response_adaptor = pimpl_->client.call("simGetImages", 
        MultirotorRpcLibAdapators::ImageRequest::from(request))
        .as<vector<MultirotorRpcLibAdapators::ImageResponse>>();

    return MultirotorRpcLibAdapators::ImageResponse::to(response_adaptor);
}
vector<uint8_t> MultirotorRpcLibClient::simGetImage(int camera_id, VehicleCameraBase::ImageType type)
{
    vector<uint8_t> result = pimpl_->client.call("simGetImage", camera_id, type).as<vector<uint8_t>>();
    if (result.size() == 1) {
        // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
        result.clear();
    }
    return result;
}

//status getters
Vector3r MultirotorRpcLibClient::getPosition()
{
    return pimpl_->client.call("getPosition").as<MultirotorRpcLibAdapators::Vector3r>().to();
}
Vector3r MultirotorRpcLibClient::getVelocity()
{
    return pimpl_->client.call("getVelocity").as<MultirotorRpcLibAdapators::Vector3r>().to();
}
Quaternionr MultirotorRpcLibClient::getOrientation()
{
    return pimpl_->client.call("getOrientation").as<MultirotorRpcLibAdapators::Quaternionr>().to();
}

DroneControllerBase::LandedState MultirotorRpcLibClient::getLandedState()
{
    int result = pimpl_->client.call("getLandedState").as<int>();
    return static_cast<DroneControllerBase::LandedState>(result);
}

RCData MultirotorRpcLibClient::getRCData()
{
    return pimpl_->client.call("getRCData").as<MultirotorRpcLibAdapators::RCData>().to();
}

CollisionInfo MultirotorRpcLibClient::getCollisionInfo()
{
    return pimpl_->client.call("getCollisionInfo").as<MultirotorRpcLibAdapators::CollisionInfo>().to();
}

TTimePoint MultirotorRpcLibClient::timestampNow()
{
    return pimpl_->client.call("timestampNow").as<TTimePoint>();
}

GeoPoint MultirotorRpcLibClient::getHomeGeoPoint()
{
    return pimpl_->client.call("getHomeGeoPoint").as<MultirotorRpcLibAdapators::GeoPoint>().to();
}

GeoPoint MultirotorRpcLibClient::getGpsLocation()
{
    return pimpl_->client.call("getGpsLocation").as<MultirotorRpcLibAdapators::GeoPoint>().to();
}

bool MultirotorRpcLibClient::isApiControlEnabled()
{
    return pimpl_->client.call("isApiControlEnabled").as<bool>();
}

bool MultirotorRpcLibClient::isSimulationMode()
{
    return pimpl_->client.call("isSimulationMode").as<bool>();
}

std::string MultirotorRpcLibClient::getDebugInfo()
{
    return pimpl_->client.call("getServerDebugInfo").as<std::string>();
}

void MultirotorRpcLibClient::confirmConnection()
{
    ClockBase* clock = ClockFactory::get();

    // make sure we can talk to the DroneServer
    //std::cout << "Contacting DroneServer..." << std::flush;
    //command_context.client.ping();
    //std::cout << "DroneServer is responding." << std::endl;

    std::cout << "Waiting for connection - " << std::flush;
    const TTimeDelta pause_time = 1;
    while (getConnectionState() != MultirotorRpcLibClient::ConnectionState::Connected)
    {
        std::cout << "X" << std::flush;
        clock->sleep_for(pause_time); 
    }
    std::cout << std::endl << "Connected!" << std::endl;

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


}} //namespace

#endif
#endif

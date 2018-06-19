// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibClientBase.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <functional>
#include <vector>
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#ifdef nil
#undef nil
#endif // nil
#include "rpc/client.h"
#include "api/RpcLibAdapatorsBase.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {

struct RpcLibClientBase::impl {
    impl(const string&  ip_address, uint16_t port, float timeout_sec)
        : client(ip_address, port)
    {
        // some long flight path commands can take a while, so we give it up to 1 hour max.
        client.set_timeout(static_cast<int64_t>(timeout_sec * 1.0E3));
    }

    rpc::client client;
};

typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;

RpcLibClientBase::RpcLibClientBase(const string&  ip_address, uint16_t port, float timeout_sec)
{
    pimpl_.reset(new impl(ip_address, port, timeout_sec));
}

RpcLibClientBase::~RpcLibClientBase()
{}

bool RpcLibClientBase::ping()
{
    return pimpl_->client.call("ping").as<bool>();
}
RpcLibClientBase::ConnectionState RpcLibClientBase::getConnectionState()
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
void RpcLibClientBase::enableApiControl(bool is_enabled, const std::string& vehicle_name)
{
    pimpl_->client.call("enableApiControl", is_enabled, vehicle_name);
}
bool RpcLibClientBase::isApiControlEnabled(const std::string& vehicle_name) const
{
    return pimpl_->client.call("isApiControlEnabled", vehicle_name).as<bool>();
}
int RpcLibClientBase::getClientVersion() const
{
    return 1; //sync with Python client
}
int RpcLibClientBase::getMinRequiredServerVersion() const
{
    return 1; //sync with Python client
}
int RpcLibClientBase::getMinRequiredClientVersion() const
{
    return pimpl_->client.call("getMinRequiredClientVersion").as<int>();
}
int RpcLibClientBase::getServerVersion() const
{
    return pimpl_->client.call("getServerVersion").as<int>();
}

void RpcLibClientBase::reset()
{
    pimpl_->client.call("reset");
}

void RpcLibClientBase::confirmConnection()
{
    ClockBase* clock = ClockFactory::get();

    // make sure we can talk to the DroneServer
    //std::cout << "Contacting DroneServer..." << std::flush;
    //command_context.client.ping();
    //std::cout << "DroneServer is responding." << std::endl;

    std::cout << "Waiting for connection - " << std::flush;
    const TTimeDelta pause_time = 1;
    while (getConnectionState() != RpcLibClientBase::ConnectionState::Connected)
    {
        std::cout << "X" << std::flush;
        clock->sleep_for(pause_time); 
    }
    std::cout << std::endl << "Connected!" << std::endl;

    auto server_ver = getServerVersion();
    auto client_ver = getClientVersion();
    auto server_min_ver = getMinRequiredServerVersion();
    auto client_min_ver = getMinRequiredClientVersion();
    
    std::string ver_info = Utils::stringf("Client Ver:%i (Min Req:%i), Server Ver:%i (Min Req:%i)",
        client_ver, client_min_ver, server_ver, server_min_ver);

    if (server_ver < server_min_ver) {
        std::cerr << std::endl << ver_info << std::endl;
        std::cerr << std::endl << "AirSim server is of older version and not supported by this client. Please upgrade!" << std::endl;
    }
    else if (client_ver < client_min_ver) {
        std::cerr << std::endl << ver_info << std::endl;
        std::cerr << std::endl << "AirSim client is of older version and not supported by this server. Please upgrade!" << std::endl;
    }
    else
        std::cout << std::endl << ver_info << std::endl;
}

bool RpcLibClientBase::armDisarm(bool arm, const std::string& vehicle_name)
{
    return pimpl_->client.call("armDisarm", arm, vehicle_name).as<bool>();
}

msr::airlib::GeoPoint RpcLibClientBase::getHomeGeoPoint(const std::string& vehicle_name) const
{
    return pimpl_->client.call("getHomeGeoPoint", vehicle_name).as<RpcLibAdapatorsBase::GeoPoint>().to();
}

bool RpcLibClientBase::simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
    return pimpl_->client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex).as<bool>();
}
int RpcLibClientBase::simGetSegmentationObjectID(const std::string& mesh_name) const
{
    return pimpl_->client.call("simGetSegmentationObjectID", mesh_name).as<int>();
}

CollisionInfo RpcLibClientBase::simGetCollisionInfo(const std::string& vehicle_name) const
{
    return pimpl_->client.call("getCollisionInfo", vehicle_name).as<RpcLibAdapatorsBase::CollisionInfo>().to();
}


//sim only
Pose RpcLibClientBase::simGetVehiclePose(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetVehiclePose", vehicle_name).as<RpcLibAdapatorsBase::Pose>().to();
}
void RpcLibClientBase::simSetVehiclePose(const Pose& pose, bool ignore_collision, const std::string& vehicle_name)
{
    pimpl_->client.call("simSetVehiclePose", RpcLibAdapatorsBase::Pose(pose), ignore_collision, vehicle_name);
}

vector<ImageCaptureBase::ImageResponse> RpcLibClientBase::simGetImages(vector<ImageCaptureBase::ImageRequest> request, const std::string& vehicle_name)
{
    const auto& response_adaptor = pimpl_->client.call("simGetImages", 
        RpcLibAdapatorsBase::ImageRequest::from(request), vehicle_name)
        .as<vector<RpcLibAdapatorsBase::ImageResponse>>();

    return RpcLibAdapatorsBase::ImageResponse::to(response_adaptor);
}
vector<uint8_t> RpcLibClientBase::simGetImage(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name)
{
    vector<uint8_t> result = pimpl_->client.call("simGetImage", camera_name, type, vehicle_name).as<vector<uint8_t>>();
    if (result.size() == 1) {
        // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
        result.clear();
    }
    return result;
}

void RpcLibClientBase::simPrintLogMessage(const std::string& message, std::string message_param, unsigned char  severity)
{
    pimpl_->client.call("simPrintLogMessage", message, message_param, severity);
}

bool RpcLibClientBase::simIsPaused() const
{
    return pimpl_->client.call("simIsPaused").as<bool>();
}

void RpcLibClientBase::simPause(bool is_paused)
{
    pimpl_->client.call("simPause", is_paused);
}

void RpcLibClientBase::simContinueForTime(double seconds)
{
    pimpl_->client.call("simContinueForTime", seconds);
}

msr::airlib::Pose RpcLibClientBase::simGetObjectPose(const std::string& object_name) const
{
    return pimpl_->client.call("simGetObjectPose", object_name).as<RpcLibAdapatorsBase::Pose>().to();
}

CameraInfo RpcLibClientBase::simGetCameraInfo(const std::string& camera_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetCameraInfo", camera_name, vehicle_name).as<RpcLibAdapatorsBase::CameraInfo>().to();
}
void RpcLibClientBase::simSetCameraOrientation(const std::string& camera_name, const Quaternionr& orientation, const std::string& vehicle_name)
{
    pimpl_->client.call("simSetCameraOrientation", camera_name, RpcLibAdapatorsBase::Quaternionr(orientation), vehicle_name);
}

msr::airlib::Kinematics::State RpcLibClientBase::simGetGroundTruthKinematics(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetGroundTruthKinematics", vehicle_name).as<RpcLibAdapatorsBase::KinematicsState>().to();
}
msr::airlib::Environment::State RpcLibClientBase::simGetGroundTruthEnvironment(const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetGroundTruthEnvironment", vehicle_name).as<RpcLibAdapatorsBase::EnvironmentState>().to();;
}

void RpcLibClientBase::cancelLastTask(const std::string& vehicle_name)
{
    pimpl_->client.call("cancelLastTask", vehicle_name);
}

//return value of last task. It should be true if task completed without
//cancellation or timeout
RpcLibClientBase* RpcLibClientBase::waitOnLastTask(bool* task_result, float timeout_sec)
{
    //should be implemented by derived class if it supports async task,
    //for example using futures
    unused(timeout_sec);
    unused(task_result);

    return this;
}

void* RpcLibClientBase::getClient()
{
    return &pimpl_->client;
}
const void* RpcLibClientBase::getClient() const
{
    return &pimpl_->client;
}

}} //namespace

#endif
#endif

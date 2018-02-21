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
    impl(const string&  ip_address, uint16_t port, uint timeout_ms)
        : client(ip_address, port)
    {
        // some long flight path commands can take a while, so we give it up to 1 hour max.
        client.set_timeout(timeout_ms);
    }

    rpc::client client;
};

typedef msr::airlib_rpclib::RpcLibAdapatorsBase RpcLibAdapatorsBase;

RpcLibClientBase::RpcLibClientBase(const string&  ip_address, uint16_t port, uint timeout_ms)
{
    pimpl_.reset(new impl(ip_address, port, timeout_ms));
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
bool RpcLibClientBase::simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
    return pimpl_->client.call("simSetSegmentationObjectID", mesh_name, object_id, is_name_regex).as<bool>();
}
int RpcLibClientBase::simGetSegmentationObjectID(const std::string& mesh_name)
{
    return pimpl_->client.call("simGetSegmentationObjectID", mesh_name).as<int>();
}
void RpcLibClientBase::enableApiControl(bool is_enabled)
{
    pimpl_->client.call("enableApiControl", is_enabled);
}
bool RpcLibClientBase::isApiControlEnabled()
{
    return pimpl_->client.call("isApiControlEnabled").as<bool>();
}

//sim only
void RpcLibClientBase::simSetPose(const Pose& pose, bool ignore_collision)
{
    pimpl_->client.call("simSetPose", RpcLibAdapatorsBase::Pose(pose), ignore_collision);
}
Pose RpcLibClientBase::simGetPose()
{
    return pimpl_->client.call("simGetPose").as<RpcLibAdapatorsBase::Pose>().to();
}
vector<ImageCaptureBase::ImageResponse> RpcLibClientBase::simGetImages(vector<ImageCaptureBase::ImageRequest> request)
{
    const auto& response_adaptor = pimpl_->client.call("simGetImages", 
        RpcLibAdapatorsBase::ImageRequest::from(request))
        .as<vector<RpcLibAdapatorsBase::ImageResponse>>();

    return RpcLibAdapatorsBase::ImageResponse::to(response_adaptor);
}
vector<uint8_t> RpcLibClientBase::simGetImage(int camera_id, ImageCaptureBase::ImageType type)
{
    vector<uint8_t> result = pimpl_->client.call("simGetImage", camera_id, type).as<vector<uint8_t>>();
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


msr::airlib::GeoPoint RpcLibClientBase::getHomeGeoPoint()
{
    return pimpl_->client.call("getHomeGeoPoint").as<RpcLibAdapatorsBase::GeoPoint>().to();
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
}

void* RpcLibClientBase::getClient()
{
    return &pimpl_->client;
}

CameraInfo RpcLibClientBase::getCameraInfo(int camera_id)
{
    return pimpl_->client.call("getCameraInfo", camera_id).as<RpcLibAdapatorsBase::CameraInfo>().to();
}

void RpcLibClientBase::setCameraOrientation(int camera_id, const Quaternionr& orientation)
{
    pimpl_->client.call("setCameraOrientation", camera_id, RpcLibAdapatorsBase::Quaternionr(orientation));
}

CollisionInfo RpcLibClientBase::getCollisionInfo()
{
    return pimpl_->client.call("getCollisionInfo").as<RpcLibAdapatorsBase::CollisionInfo>().to();
}

msr::airlib::Pose RpcLibClientBase::simGetObjectPose(const std::string& object_name)
{
    return pimpl_->client.call("simGetObjectPose", object_name).as<RpcLibAdapatorsBase::Pose>().to();
}


}} //namespace

#endif
#endif

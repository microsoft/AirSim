// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/car/api/CarRpcLibClient.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#undef check
#include "rpc/client.h"
#include "vehicles/car/api/CarRpcLibAdapators.hpp"
STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif			  


namespace msr { namespace airlib {

struct CarRpcLibClient::impl {
    impl(const string&  ip_address, uint16_t port, uint timeout_ms)
        : client(ip_address, port)
    {
        // some long flight path commands can take a while, so we give it up to 1 hour max.
        client.set_timeout(timeout_ms);
    }

    rpc::client client;
};

typedef msr::airlib_rpclib::CarRpcLibAdapators CarRpcLibAdapators;

CarRpcLibClient::CarRpcLibClient(const string&  ip_address, uint16_t port, uint timeout_ms)
{
    pimpl_.reset(new impl(ip_address, port, timeout_ms));
}

CarRpcLibClient::~CarRpcLibClient()
{}

bool CarRpcLibClient::ping()
{
    return pimpl_->client.call("ping").as<bool>();
}
CarRpcLibClient::ConnectionState CarRpcLibClient::getConnectionState()
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
void CarRpcLibClient::enableApiControl(bool is_enabled)
{
    pimpl_->client.call("enableApiControl", is_enabled);
}
bool CarRpcLibClient::isApiControlEnabled()
{
    return pimpl_->client.call("isApiControlEnabled").as<bool>();
}

//sim only
void CarRpcLibClient::simSetPose(const Pose& pose, bool ignore_collison)
{
    pimpl_->client.call("simSetPose", CarRpcLibAdapators::Pose(pose), ignore_collison);
}
Pose CarRpcLibClient::simGetPose()
{
    return pimpl_->client.call("simGetPose").as<CarRpcLibAdapators::Pose>().to();
}
vector<VehicleCameraBase::ImageResponse> CarRpcLibClient::simGetImages(vector<VehicleCameraBase::ImageRequest> request)
{
    const auto& response_adaptor = pimpl_->client.call("simGetImages", 
        CarRpcLibAdapators::ImageRequest::from(request))
        .as<vector<CarRpcLibAdapators::ImageResponse>>();

    return CarRpcLibAdapators::ImageResponse::to(response_adaptor);
}
vector<uint8_t> CarRpcLibClient::simGetImage(int camera_id, VehicleCameraBase::ImageType type)
{
    vector<uint8_t> result = pimpl_->client.call("simGetImage", camera_id, type).as<vector<uint8_t>>();
    if (result.size() == 1) {
        // rpclib has a bug with serializing empty vectors, so we return a 1 byte vector instead.
        result.clear();
    }
    return result;
}

void CarRpcLibClient::setCarControls(const CarApiBase::CarControls& controls)
{
    pimpl_->client.call("setCarControls", CarRpcLibAdapators::CarControls(controls));
}

void CarRpcLibClient::reset()
{
    pimpl_->client.call("reset");
}

CarApiBase::CarState CarRpcLibClient::getCarState()
{
    return pimpl_->client.call("getCarState").as<CarRpcLibAdapators::CarState>().to();

}

msr::airlib::GeoPoint CarRpcLibClient::getHomeGeoPoint()
{
    return pimpl_->client.call("getHomeGeoPoint").as<CarRpcLibAdapators::GeoPoint>().to();
}

void CarRpcLibClient::confirmConnection()
{
    ClockBase* clock = ClockFactory::get();

    // make sure we can talk to the DroneServer
    //std::cout << "Contacting DroneServer..." << std::flush;
    //command_context.client.ping();
    //std::cout << "DroneServer is responding." << std::endl;

    std::cout << "Waiting for connection - " << std::flush;
    const TTimeDelta pause_time = 1;
    while (getConnectionState() != CarRpcLibClient::ConnectionState::Connected)
    {
        std::cout << "X" << std::flush;
        clock->sleep_for(pause_time); 
    }
    std::cout << std::endl << "Connected!" << std::endl;
}


}} //namespace

#endif
#endif

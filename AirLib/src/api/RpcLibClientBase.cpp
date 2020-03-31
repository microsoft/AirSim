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

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

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

msr::airlib::LidarData RpcLibClientBase::getLidarData(const std::string& lidar_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getLidarData", lidar_name, vehicle_name).as<RpcLibAdapatorsBase::LidarData>().to();
}

msr::airlib::ImuBase::Output RpcLibClientBase::getImuData(const std::string& imu_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getImuData", imu_name, vehicle_name).as<RpcLibAdapatorsBase::ImuData>().to();
}

msr::airlib::BarometerBase::Output RpcLibClientBase::getBarometerData(const std::string& barometer_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getBarometerData", barometer_name, vehicle_name).as<RpcLibAdapatorsBase::BarometerData>().to();
}

msr::airlib::MagnetometerBase::Output RpcLibClientBase::getMagnetometerData(const std::string& magnetometer_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getMagnetometerData", magnetometer_name, vehicle_name).as<RpcLibAdapatorsBase::MagnetometerData>().to();
}

msr::airlib::GpsBase::Output RpcLibClientBase::getGpsData(const std::string& gps_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getGpsData", gps_name, vehicle_name).as<RpcLibAdapatorsBase::GpsData>().to();
}

msr::airlib::DistanceBase::Output RpcLibClientBase::getDistanceSensorData(const std::string& distance_sensor_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("getDistanceSensorData", distance_sensor_name, vehicle_name).as<RpcLibAdapatorsBase::DistanceSensorData>().to();
}

vector<int> RpcLibClientBase::simGetLidarSegmentation(const std::string& lidar_name, const std::string& vehicle_name) const
{
    return pimpl_->client.call("simGetLidarSegmentation", lidar_name, vehicle_name).as<vector<int>>();
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
    return pimpl_->client.call("simGetCollisionInfo", vehicle_name).as<RpcLibAdapatorsBase::CollisionInfo>().to();
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

void RpcLibClientBase::simFlushPersistentMarkers()
{
    pimpl_->client.call("simFlushPersistentMarkers");
}

void RpcLibClientBase::simPlotPoints(const vector<Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent)
{
    vector<RpcLibAdapatorsBase::Vector3r> conv_points;
    RpcLibAdapatorsBase::from(points, conv_points);
    pimpl_->client.call("simPlotPoints", conv_points, color_rgba, size, duration, is_persistent);
}

void RpcLibClientBase::simPlotLineStrip(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    vector<RpcLibAdapatorsBase::Vector3r> conv_points;
    RpcLibAdapatorsBase::from(points, conv_points);
    pimpl_->client.call("simPlotLineStrip", conv_points, color_rgba, thickness, duration, is_persistent);
}

void RpcLibClientBase::simPlotLineList(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    vector<RpcLibAdapatorsBase::Vector3r> conv_points;
    RpcLibAdapatorsBase::from(points, conv_points);
    pimpl_->client.call("simPlotLineList", conv_points, color_rgba, thickness, duration, is_persistent);
}

void RpcLibClientBase::simPlotArrows(const vector<Vector3r>& points_start, const vector<Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent)
{
    vector<RpcLibAdapatorsBase::Vector3r> conv_points_start;
    RpcLibAdapatorsBase::from(points_start, conv_points_start);
    vector<RpcLibAdapatorsBase::Vector3r> conv_points_end;
    RpcLibAdapatorsBase::from(points_end, conv_points_end);
    pimpl_->client.call("simPlotArrows", conv_points_start, conv_points_end, color_rgba, thickness, arrow_size, duration, is_persistent);
}

void RpcLibClientBase::simPlotStrings(const vector<std::string>& strings, const vector<Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration)
{
    vector<RpcLibAdapatorsBase::Vector3r> conv_positions;
    RpcLibAdapatorsBase::from(positions, conv_positions);
    pimpl_->client.call("simPlotStrings", strings, conv_positions, scale, color_rgba, duration);
}

void RpcLibClientBase::simPlotTransforms(const vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent)
{
    vector<RpcLibAdapatorsBase::Pose> conv_poses;
    RpcLibAdapatorsBase::from(poses, conv_poses);
    pimpl_->client.call("simPlotTransforms", conv_poses, scale, thickness, duration, is_persistent);
}

void RpcLibClientBase::simPlotTransformsWithNames(const vector<Pose>& poses, const vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration)
{
    vector<RpcLibAdapatorsBase::Pose> conv_poses;
    RpcLibAdapatorsBase::from(poses, conv_poses);
    pimpl_->client.call("simPlotTransformsWithNames", conv_poses, names, tf_scale, tf_thickness, text_scale, text_color_rgba, duration);

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

void RpcLibClientBase::simEnableWeather(bool enable)
{
    pimpl_->client.call("simEnableWeather", enable);
}
void RpcLibClientBase::simSetWeatherParameter(WorldSimApiBase::WeatherParameter param, float val)
{
    pimpl_->client.call("simSetWeatherParameter", param, val);
}

void RpcLibClientBase::simSetTimeOfDay(bool is_enabled, const string& start_datetime, bool is_start_datetime_dst,
    float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    pimpl_->client.call("simSetTimeOfDay", is_enabled, start_datetime, is_start_datetime_dst,
        celestial_clock_speed, update_interval_secs, move_sun);
}

vector<string> RpcLibClientBase::simListSceneObjects(const string& name_regex) const
{
    return pimpl_->client.call("simListSceneObjects", name_regex).as<vector<string>>();
}

std::vector<std::string> RpcLibClientBase::simSwapTextures(const std::string& tags, int tex_id, int component_id, int material_id)
{
	return pimpl_->client.call("simSwapTextures", tags, tex_id, component_id, material_id).as<vector<string>>();
}

msr::airlib::Pose RpcLibClientBase::simGetObjectPose(const std::string& object_name) const
{
    return pimpl_->client.call("simGetObjectPose", object_name).as<RpcLibAdapatorsBase::Pose>().to();
}
bool RpcLibClientBase::simSetObjectPose(const std::string& object_name, const msr::airlib::Pose& pose, bool teleport)
{
    return pimpl_->client.call("simSetObjectPose", object_name, RpcLibAdapatorsBase::Pose(pose), teleport).as<bool>();
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

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibClient_hpp
#define air_RpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "controllers/DroneCommon.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "safety/SafetyEval.hpp"

namespace msr { namespace airlib {

class RpcLibClient {
public:
    enum class ConnectionState {
        Initial, Connected, Disconnected, Reset, Unknown
    };
public:
    RpcLibClient(const string& ip_address = "localhost", uint16_t port = 41451);
    ConnectionState getConnectionState();
    bool ping();
    bool armDisarm(bool arm);
    void setOffboardMode(bool is_set);
    void setSimulationMode(bool is_set);
    void start();
    void stop();
    bool takeoff(float max_wait_ms = 15);
    bool land(float max_wait_seconds = 60);
    bool goHome();
    bool moveByAngle(float pitch, float roll, float z, float yaw, float duration);

    bool moveByVelocity(float vx, float vy, float vz, float duration, 
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode());
    bool moveByVelocityZ(float vx, float vy, float z, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode());
    bool moveOnPath(const vector<Vector3r>& path, float velocity, float max_wait_seconds = 60,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    bool moveToPosition(float x, float y, float z, float velocity, float max_wait_seconds = 60,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    bool moveToZ(float z, float velocity, float max_wait_seconds = 60,
        const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    bool moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode);
    bool rotateToYaw(float yaw, float max_wait_seconds = 60, float margin = 5);
    bool rotateByYawRate(float yaw_rate, float duration);
    bool hover();

    Vector3r getPosition();
    CollisionInfo getCollisionInfo();
    Vector3r getVelocity();
    Quaternionr getOrientation();
    DroneControllerBase::LandedState getLandedState();
    RCData getRCData();
    TTimePoint timestampNow();
    GeoPoint getHomePoint();
    GeoPoint getGpsLocation();
    bool isOffboardMode();
    bool isSimulationMode();
    std::string getDebugInfo();

    //request image
    vector<uint8_t> getImageForCamera(int camera_id, VehicleCamera::ImageType type);

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);

    ~RpcLibClient();    //required for pimpl
private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibClient_hpp
#define air_MultirotorRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "controllers/VehicleCameraBase.hpp"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "vehicles/multirotor/controllers/DroneCommon.hpp"

namespace msr { namespace airlib {

class MultirotorRpcLibClient {
public:
    enum class ConnectionState : uint {
        Initial = 0, Connected, Disconnected, Reset, Unknown
    };
public:
    MultirotorRpcLibClient(const string& ip_address = "localhost", uint16_t port = 41451, uint timeout_ms = 60000);
    ConnectionState getConnectionState();
    bool ping();
    bool armDisarm(bool arm);
    void enableApiControl(bool is_enabled);
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
    bool moveByManual(float vx_max, float vy_max, float z_min, float duration, 
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode());
    bool rotateToYaw(float yaw, float max_wait_seconds = 60, float margin = 5);
    bool rotateByYawRate(float yaw_rate, float duration);
    bool hover();

    void simSetPose(const Pose& pose, bool ignore_collison);
    Pose simGetPose();

    vector<VehicleCameraBase::ImageResponse> simGetImages(vector<VehicleCameraBase::ImageRequest> request);
    vector<uint8_t> simGetImage(int camera_id, VehicleCameraBase::ImageType type);
    
    Vector3r getPosition();
    CollisionInfo getCollisionInfo();
    Vector3r getVelocity();
    Quaternionr getOrientation();
    RCData getRCData();
    GeoPoint getGpsLocation();
    GeoPoint getHomeGeoPoint();

    bool isApiControlEnabled();
    bool isSimulationMode();
    std::string getDebugInfo();
    void confirmConnection();
    DroneControllerBase::LandedState getLandedState();
    TTimePoint timestampNow();

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);

    ~MultirotorRpcLibClient();    //required for pimpl
private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

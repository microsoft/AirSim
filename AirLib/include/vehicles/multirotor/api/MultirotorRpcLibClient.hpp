// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibClient_hpp
#define air_MultirotorRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "vehicles/multirotor/controllers/DroneCommon.hpp"

namespace msr { namespace airlib {

class MultirotorRpcLibClient : public RpcLibClientBase {
public:
    MultirotorRpcLibClient(const string& ip_address = "localhost", uint16_t port = 41451, uint timeout_ms = 60000);

    bool armDisarm(bool arm);
    void setSimulationMode(bool is_set);
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

    MultirotorState getMultirotorState();
    Vector3r getPosition();
    Vector3r getVelocity();
    Quaternionr getOrientation();
    GeoPoint getGpsLocation();
    bool isSimulationMode();
    std::string getDebugInfo();

    RCData getRCData();
    void setRCData(const RCData& rc_data);

    DroneControllerBase::LandedState getLandedState();

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);

    virtual ~MultirotorRpcLibClient();    //required for pimpl

};

}} //namespace
#endif

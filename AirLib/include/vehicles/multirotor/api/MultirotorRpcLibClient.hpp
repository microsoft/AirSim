// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibClient_hpp
#define air_MultirotorRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"

namespace msr { namespace airlib {

class MultirotorRpcLibClient : public RpcLibClientBase {
public:
    MultirotorRpcLibClient(const string& ip_address = "localhost", uint16_t port = 41451, float timeout_sec = 60);

    void takeoffAsync(float timeout_sec = 20);
    void landAsync(float timeout_sec = 60);
    void goHomeAsync(float timeout_sec = Utils::max<float>());

    void moveByAngleZAsync(float pitch, float roll, float z, float yaw, float duration);
    void moveByAngleThrottleAsync(float pitch, float roll, float throttle, float yaw_rate, float duration);
    void moveByVelocityAsync(float vx, float vy, float vz, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode());
    void moveByVelocityZAsync(float vx, float vy, float z, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode());
    void moveOnPathAsync(const vector<Vector3r>& path, float velocity, float timeout_sec = Utils::max<float>(),
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1);
    void moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec = Utils::max<float>(),
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1);
    void moveToZAsync(float z, float velocity, float timeout_sec = Utils::max<float>(),
        const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    void moveByManualAsync(float vx_max, float vy_max, float z_min, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode());
    void rotateToYawAsync(float yaw, float timeout_sec = Utils::max<float>(), float margin = 5);
    void rotateByYawRateAsync(float yaw_rate, float duration);
    void hoverAsync();

    void moveByRC(const RCData& rc_data);


    MultirotorState getMultirotorState();

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);

    virtual bool waitOnLastTask(float timeout_sec = Utils::nan<float>()) override;

    virtual ~MultirotorRpcLibClient();    //required for pimpl

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

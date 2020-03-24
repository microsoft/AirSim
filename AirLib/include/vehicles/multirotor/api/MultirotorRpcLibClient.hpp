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
    MultirotorRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

    MultirotorRpcLibClient* takeoffAsync(float timeout_sec = 20, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* landAsync(float timeout_sec = 60, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* goHomeAsync(float timeout_sec = Utils::max<float>(), const std::string& vehicle_name = "");

    MultirotorRpcLibClient* moveByMotorPWMsAsync(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawZAsync(float roll, float pitch, float yaw, float z, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawThrottleAsync(float roll, float pitch, float yaw, float throttle, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawrateThrottleAsync(float roll, float pitch, float yaw_rate, float throttle, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawrateZAsync(float roll, float pitch, float yaw_rate, float z, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByAngleRatesZAsync(float roll_rate, float pitch_rate, float yaw_rate, float z, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByAngleRatesThrottleAsync(float roll_rate, float pitch_rate, float yaw_rate, float throttle, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByVelocityAsync(float vx, float vy, float vz, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByVelocityZAsync(float vx, float vy, float z, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveOnPathAsync(const vector<Vector3r>& path, float velocity, float timeout_sec = Utils::max<float>(),
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec = Utils::max<float>(),
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveToZAsync(float z, float velocity, float timeout_sec = Utils::max<float>(),
        const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByManualAsync(float vx_max, float vy_max, float z_min, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* rotateToYawAsync(float yaw, float timeout_sec = Utils::max<float>(), float margin = 5, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* rotateByYawRateAsync(float yaw_rate, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* hoverAsync(const std::string& vehicle_name = "");

    void setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void moveByRC(const RCData& rc_data, const std::string& vehicle_name = "");

    MultirotorState getMultirotorState(const std::string& vehicle_name = "");

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z, const std::string& vehicle_name = "");

    virtual MultirotorRpcLibClient* waitOnLastTask(bool* task_result = nullptr, float timeout_sec = Utils::nan<float>()) override;

    virtual ~MultirotorRpcLibClient();    //required for pimpl

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

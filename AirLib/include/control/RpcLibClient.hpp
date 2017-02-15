// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibClient_hpp
#define air_RpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "DroneControlCommon.hpp"
#include "SafetyEval.hpp"
#include "DroneControlBase.hpp"

namespace msr { namespace airlib {

class RpcLibClient {
public:
    RpcLibClient(const string& ip_address = "127.0.0.1", uint16_t port = 41451);
    bool armDisarm(bool arm);
    bool requestControl();
    bool releaseControl();
    bool takeoff(float max_wait = 10);
    bool land();
    bool goHome();
    bool moveByAngle(float pitch, float roll, float z, float yaw, float duration);

    bool moveByVelocity(float vx, float vy, float vz, float duration, 
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedome, const YawMode& yaw_mode = YawMode());
    bool moveByVelocityZ(float vx, float vy, float z, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedome, const YawMode& yaw_mode = YawMode());
    bool moveOnPath(const vector<Vector3r>& path, float velocity, 
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedome, const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    bool moveToPosition(float x, float y, float z, float velocity, 
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedome, const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    bool moveToZ(float z, float velocity, 
        const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1);
    bool moveByManual(float vx_max, float vy_max, float z_min, DrivetrainType drivetrain, const YawMode& yaw_mode, float duration);
    bool rotateToYaw(float yaw, float margin = 5);
    bool rotateByYawRate(float yaw_rate, float duration);
    bool hover(float duration);
    bool sleep(float duration);
    Vector3r getPosition();
    Vector3r getVelocity();
    Quaternionr getOrientation();
    RCData getRCData();
    double timestampNow();
    GeoPoint getHomePoint();
    GeoPoint getGpsLocation();
    bool isOffboardMode();
    std::string getDebugInfo();
    void cancelAllTasks();

    //request image
    bool setImageTypeForCamera(int camera_id, DroneControlBase::ImageType type);
    DroneControlBase::ImageType getImageTypeForCamera(int camera_id);
    //get/set image
    vector<uint8_t> getImageForCamera(int camera_id, DroneControlBase::ImageType type);

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);

    ~RpcLibClient();    //required for pimpl
private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

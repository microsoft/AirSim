// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibClientBase_hpp
#define air_RpcLibClientBase_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/distance/DistanceBase.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "api/WorldSimApiBase.hpp"

namespace msr { namespace airlib {

//common methods for RCP clients of different vehicles
class RpcLibClientBase {
public:
    enum class ConnectionState : uint {
        Initial = 0, Connected, Disconnected, Reset, Unknown
    };
public:
    RpcLibClientBase(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);
    virtual ~RpcLibClientBase();    //required for pimpl

    void confirmConnection();
    void reset();

    ConnectionState getConnectionState();
    bool ping();
    int getClientVersion() const;
    int getServerVersion() const;
    int getMinRequiredServerVersion() const;
    int getMinRequiredClientVersion() const;

    bool simIsPaused() const;
    void simPause(bool is_paused);
    void simContinueForTime(double seconds);

    void simSetTimeOfDay(bool is_enabled, const string& start_datetime = "", bool is_start_datetime_dst = false,
        float celestial_clock_speed = 1, float update_interval_secs = 60, bool move_sun = true);

    void simEnableWeather(bool enable);
    void simSetWeatherParameter(WorldSimApiBase::WeatherParameter param, float val);

    vector<string> simListSceneObjects(const string& name_regex = string(".*")) const;
    Pose simGetObjectPose(const std::string& object_name) const;
    bool simSetObjectPose(const std::string& object_name, const Pose& pose, bool teleport = true);
    
    //task management APIs
    void cancelLastTask(const std::string& vehicle_name = "");
    virtual RpcLibClientBase* waitOnLastTask(bool* task_result = nullptr, float timeout_sec = Utils::nan<float>());

    bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false);
    int simGetSegmentationObjectID(const std::string& mesh_name) const;
    void simPrintLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0);


    bool armDisarm(bool arm, const std::string& vehicle_name = "");
    bool isApiControlEnabled(const std::string& vehicle_name = "") const;
    void enableApiControl(bool is_enabled, const std::string& vehicle_name = "");

    msr::airlib::GeoPoint getHomeGeoPoint(const std::string& vehicle_name = "") const;

    // sensor APIs
    msr::airlib::LidarData getLidarData(const std::string& lidar_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::ImuBase::Output getImuData(const std::string& imu_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::BarometerBase::Output getBarometerData(const std::string& barometer_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::MagnetometerBase::Output getMagnetometerData(const std::string& magnetometer_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::GpsBase::Output getGpsData(const std::string& gps_name = "", const std::string& vehicle_name = "") const;
    msr::airlib::DistanceBase::Output getDistanceSensorData(const std::string& distance_sensor_name = "", const std::string& vehicle_name = "") const;

    // sensor omniscient APIs
    vector<int> simGetLidarSegmentation(const std::string& lidar_name = "", const std::string& vehicle_name = "") const;

    Pose simGetVehiclePose(const std::string& vehicle_name = "") const;
    void simSetVehiclePose(const Pose& pose, bool ignore_collision, const std::string& vehicle_name = "");

    vector<ImageCaptureBase::ImageResponse> simGetImages(vector<ImageCaptureBase::ImageRequest> request, const std::string& vehicle_name = "");
    vector<uint8_t> simGetImage(const std::string& camera_name, ImageCaptureBase::ImageType type, const std::string& vehicle_name = "");

    CollisionInfo simGetCollisionInfo(const std::string& vehicle_name = "") const;

    CameraInfo simGetCameraInfo(const std::string& camera_name, const std::string& vehicle_name = "") const;
    void simSetCameraOrientation(const std::string& camera_name, const Quaternionr& orientation, const std::string& vehicle_name = "");

    msr::airlib::Kinematics::State simGetGroundTruthKinematics(const std::string& vehicle_name = "") const;
    msr::airlib::Environment::State simGetGroundTruthEnvironment(const std::string& vehicle_name = "") const;

    //----------- APIs to control ACharacter in scene ----------/
    void simCharSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name = "");
    float simCharGetFaceExpression(const std::string& expression_name, const std::string& character_name = "") const;
    std::vector<std::string> simCharGetAvailableFaceExpressions();
    void simCharSetSkinDarkness(float value, const std::string& character_name = "");
    float simCharGetSkinDarkness(const std::string& character_name = "") const;
    void simCharSetSkinAgeing(float value, const std::string& character_name = "");
    float simCharGetSkinAgeing(const std::string& character_name = "") const;
    void simCharSetHeadRotation(const msr::airlib::Quaternionr& q, const std::string& character_name = "");
    msr::airlib::Quaternionr simCharGetHeadRotation(const std::string& character_name = "") const;
    void simCharSetBonePose(const std::string& bone_name, const msr::airlib::Pose& pose, const std::string& character_name = "");
    msr::airlib::Pose simCharGetBonePose(const std::string& bone_name, const std::string& character_name = "") const;
    void simCharResetBonePose(const std::string& bone_name, const std::string& character_name = "");
    void simCharSetFacePreset(const std::string& preset_name, float value, const std::string& character_name = "");
    void simSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name = "");
    void simSetBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses, const std::string& character_name = "");
    std::unordered_map<std::string, msr::airlib::Pose> simGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name = "") const;

protected:
    void* getClient();
    const void* getClient() const;

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

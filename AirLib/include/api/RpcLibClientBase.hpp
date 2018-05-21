// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibClientBase_hpp
#define air_RpcLibClientBase_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"


namespace msr { namespace airlib {

//common methods for RCP clients of different vehicles
class RpcLibClientBase {
public:
    enum class ConnectionState : uint {
        Initial = 0, Connected, Disconnected, Reset, Unknown
    };
public:
    RpcLibClientBase(const string& ip_address = "localhost", uint16_t port = 41451, uint timeout_ms = 60000);
    virtual ~RpcLibClientBase();    //required for pimpl

    void confirmConnection();
    bool isApiControlEnabled() const;
    void enableApiControl(bool is_enabled);
    void resetVehicle();
    void simResetWorld();
    bool armDisarm(bool arm);

    ConnectionState getConnectionState();
    bool ping();
    int getClientVersion() const;
    int getServerVersion() const;
    int getMinRequiredServerVersion() const;
    int getMinRequiredClientVersion() const;

    bool simIsPaused() const;
    void simPause(bool is_paused);
    void simContinueForTime(double seconds);

    msr::airlib::GeoPoint getHomeGeoPoint() const;

    Pose simGetVehiclePose() const;
    void simSetVehiclePose(const Pose& pose, bool ignore_collision);
    Pose simGetObjectPose(const std::string& object_name) const;

    vector<ImageCaptureBase::ImageResponse> simGetImages(vector<ImageCaptureBase::ImageRequest> request);
    vector<uint8_t> simGetImage(int camera_id, ImageCaptureBase::ImageType type);

    CollisionInfo simGetCollisionInfo() const;

    bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false);
    int simGetSegmentationObjectID(const std::string& mesh_name) const;
    void simPrintLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0);

    CameraInfo getCameraInfo(int camera_id) const;
    void setCameraOrientation(int camera_id, const Quaternionr& orientation);


protected:
    const void* getClient() const;
    void* getClient()
    {
        return const_cast<void*>(getClient());
    }

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

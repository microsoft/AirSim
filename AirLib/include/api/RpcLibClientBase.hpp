// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibClientBase_hpp
#define air_RpcLibClientBase_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "controllers/VehicleCameraBase.hpp"


namespace msr { namespace airlib {

class RpcLibClientBase {
public:
    enum class ConnectionState : uint {
        Initial = 0, Connected, Disconnected, Reset, Unknown
    };
public:
    RpcLibClientBase(const string& ip_address = "localhost", uint16_t port = 42451, uint timeout_ms = 60000);
    ConnectionState getConnectionState();
    bool ping();

    vector<VehicleCameraBase::ImageResponse> simGetImages(vector<VehicleCameraBase::ImageRequest> request);
    vector<uint8_t> simGetImage(int camera_id, VehicleCameraBase::ImageType type);
    msr::airlib::GeoPoint getHomeGeoPoint();

    void simSetPose(const Pose& pose, bool ignore_collision);
    Pose simGetPose();

    void confirmConnection();
    bool isApiControlEnabled();
    void enableApiControl(bool is_enabled);
    void reset();

    CollisionInfo getCollisionInfo();

    bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false);
    int simGetSegmentationObjectID(const std::string& mesh_name);
    void simPrintLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0);

    virtual ~RpcLibClientBase();    //required for pimpl

protected:
    void* getClient();

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

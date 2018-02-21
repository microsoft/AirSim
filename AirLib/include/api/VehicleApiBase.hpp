// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"

namespace msr { namespace airlib {


class VehicleApiBase {
public:
    virtual GeoPoint getHomeGeoPoint() = 0;
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() const = 0;
    virtual void reset() = 0;

    virtual vector<ImageCaptureBase::ImageResponse> simGetImages(const vector<ImageCaptureBase::ImageRequest>& request) = 0;
    virtual vector<uint8_t> simGetImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) = 0;

    virtual void simSetPose(const Pose& pose, bool ignore_collision) = 0;
    virtual Pose simGetPose() = 0;

    virtual bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
    virtual int simGetSegmentationObjectID(const std::string& mesh_name) = 0;

    virtual void simPrintLogMessage(const std::string& message, const std::string& message_param = "", unsigned char severity = 0) = 0;
    
    virtual CollisionInfo getCollisionInfo() = 0;

    virtual Pose simGetObjectPose(const std::string& object_name) = 0;

    virtual CameraInfo getCameraInfo(int camera_id) const = 0;
    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) = 0;
    
    virtual ~VehicleApiBase() = default;
};


}} //namespace
#endif

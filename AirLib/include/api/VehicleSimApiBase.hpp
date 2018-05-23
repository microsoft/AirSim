// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleSimApiBase_hpp
#define air_VehicleSimApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "physics/Kinematics.hpp"

namespace msr { namespace airlib {

class VehicleSimApiBase {
public:
    virtual ~VehicleSimApiBase() = default;

    virtual void reset() = 0;

    virtual vector<ImageCaptureBase::ImageResponse> getImages(const vector<ImageCaptureBase::ImageRequest>& request) const = 0;
    virtual vector<uint8_t> getImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) const = 0;

    virtual void setPose(const Pose& pose, bool ignore_collision) = 0;
    virtual Pose getPose() const = 0;

    virtual CollisionInfo getCollisionInfo() const = 0;
    virtual Kinematics::State getGroundTruthKinematics() const = 0;
    virtual int getRemoteControlID() const { return -1; }

    virtual CameraInfo getCameraInfo(int camera_id) const = 0;
    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) = 0;
};

} } //namespace
#endif

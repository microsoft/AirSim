// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleSimApiBase_hpp
#define air_VehicleSimApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/ImageCaptureBase.hpp"
#include "physics/Kinematics.hpp"

namespace msr { namespace airlib {

class VehicleSimApiBase : public msr::airlib::UpdatableObject {
public:
    virtual ~VehicleSimApiBase() = default;

    //this method is called at every render tick when we want to transfer state from
    //physics engine to render engine. As physics engine is halted while 
    //this happens, this method should do minimal processing
    virtual void updateRenderedState(float dt)
    {
        //derived class should override if needed
    }
    //called when render changes are required at every render tick
    virtual void updateRendering(float dt)
    {
        //derived class should override if needed
    }

    virtual const ImageCaptureBase* getImageCapture() const = 0;
    virtual ImageCaptureBase* getImageCapture()
    {
        return const_cast<ImageCaptureBase*>(getImageCapture());
    }

    virtual std::vector<ImageCaptureBase::ImageResponse> getImages(const std::vector<ImageCaptureBase::ImageRequest>& request) const = 0;
    virtual std::vector<uint8_t> getImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) const = 0;

    virtual Pose getPose() const = 0;
    virtual void setPose(const Pose& pose, bool ignore_collision) = 0;
    virtual const Kinematics::State* getGroundTruthKinematics() const = 0;

    virtual CameraInfo getCameraInfo(int camera_id) const = 0;
    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) = 0;

    virtual CollisionInfo getCollisionInfo() const = 0;
    virtual int getRemoteControlID() const { return -1; }
    virtual RCData getRCData() const = 0; //get reading from RC from simulator

};

} } //namespace
#endif

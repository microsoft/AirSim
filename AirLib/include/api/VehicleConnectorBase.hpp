// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleConnectorBase_hpp
#define air_VehicleConnectorBase_hpp

#include "VehicleApiBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "common/UpdatableObject.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"


namespace msr { namespace airlib {

class VehicleSimBridgeBase : public UpdatableObject
{
public:
    //pure abstract methods in addition to UpdatableObject

    //called when physics gets updated (must be fast, avoid rendering)
    virtual void updateRenderedState(float dt) = 0;
    //called when render changes are required
    virtual void updateRendering(float dt) = 0;

    virtual const VehicleApiBase* getVehicleApi() const = 0;
    virtual VehicleApiBase* getVehicleApi()
    {
        return const_cast<VehicleApiBase*>(getVehicleApi());
    }

    virtual ImageCaptureBase* getImageCapture() = 0;

    virtual void setPose(const Pose& pose, bool ignore_collision) = 0;
    virtual Pose getPose() const = 0;
    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) = 0;
    virtual int getSegmentationObjectID(const std::string& mesh_name) const = 0;
    virtual void printLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0) = 0;
    virtual Pose getActorPose(const std::string& actor_name) const = 0;
    virtual const Kinematics::State* getGroundTruthKinematics() const = 0;
    virtual const Environment* getGroundTruthEnvironment() const = 0;
    virtual CameraInfo getCameraInfo(int camera_id) const = 0;
    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) = 0;
};


}} //namespace
#endif
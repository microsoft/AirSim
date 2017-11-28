// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleConnectorBase_hpp
#define air_VehicleConnectorBase_hpp

#include "VehicleControllerBase.hpp"
#include "VehicleCameraBase.hpp"
#include "common/UpdatableObject.hpp"

namespace msr { namespace airlib {

class VehicleConnectorBase : public UpdatableObject
{
public:
    //pure abstract methods in addition to UpdatableObject

    //called when physics gets updated (must be fast, avoid rendering)
    virtual void updateRenderedState(float dt) = 0;
    //called when render changes are required
    virtual void updateRendering(float dt) = 0;

    //opens up channel to talk to vehicle via APIs
    virtual void startApiServer() = 0;
    virtual void stopApiServer() = 0;
    virtual bool isApiServerStarted() = 0;
    virtual VehicleControllerBase* getController() = 0;
    virtual VehicleCameraBase* getCamera(unsigned int index) = 0;
    virtual void setPose(const Pose& pose, bool ignore_collision) = 0;
    virtual Pose getPose() = 0;
    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) = 0;
    virtual int getSegmentationObjectID(const std::string& mesh_name) = 0;
    virtual void printLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0) = 0;

};


}} //namespace
#endif
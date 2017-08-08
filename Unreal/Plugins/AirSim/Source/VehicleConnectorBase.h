#pragma once

#include "controllers/VehicleCameraBase.hpp"
#include "common/UpdatableObject.hpp"
#include "controllers/VehicleControllerBase.hpp"
#include "api/ControlServerBase.hpp"


class VehicleConnectorBase : public msr::airlib::UpdatableObject
{
public:
    typedef msr::airlib::UpdatableObject UpdatableObject;

    //pure abstract methods in addition to UpdatableObject

    //called when physics gets updated (must be fast, avoid rendering)
    virtual void updateRenderedState() = 0;
    //called when render changes are required
    virtual void updateRendering(float dt) = 0;

    //opens up channel to talk to vehicle via APIs
    virtual void startApiServer() = 0;
    virtual void stopApiServer() = 0;
    virtual bool isApiServerStarted() = 0;
    virtual msr::airlib::VehicleControllerBase* getController() = 0;
    virtual msr::airlib::VehicleCameraBase* getCamera(unsigned int index = 0) = 0;
};

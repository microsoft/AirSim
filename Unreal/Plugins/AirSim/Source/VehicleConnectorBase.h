#pragma once

#include "common/UpdatableObject.hpp"
#include "controllers/VehicleControllerBase.hpp"
#include "rpc/ControlServerBase.hpp"


class VehicleConnectorBase : public msr::airlib::UpdatableObject
{
public:
    typedef msr::airlib::UpdatableObject UpdatableObject;

    //pure abstract methods in addition to UpdatableObject

    //called when game starts
    virtual void beginPlay() = 0;
    //called when game ends
    virtual void endPlay() = 0;
    //called when physics gets updated (must be fast, avoid rendering)
    virtual void updateRenderedState() = 0;
    //called when render changes are required
    virtual void updateRendering(float dt) = 0;

    //opens up channel to talk to vehicle via APIs
    virtual void startApiServer() = 0;
    virtual void stopApiServer() = 0;
    virtual bool isApiServerStarted() = 0;
    virtual msr::airlib::VehicleControllerBase* getController() = 0;
};

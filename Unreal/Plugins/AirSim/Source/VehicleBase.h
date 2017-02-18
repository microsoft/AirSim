#pragma once

#include "common/UpdatableObject.hpp"

class VehicleBase : public msr::airlib::UpdatableObject
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
    virtual void updateRendering() = 0;
};

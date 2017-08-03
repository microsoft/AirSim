// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_UpdatableObject_hpp
#define airsim_core_UpdatableObject_hpp

#include "common/Common.hpp"
#include "StateReporter.hpp"
#include "ClockFactory.hpp"

namespace msr { namespace airlib {

class UpdatableObject {
public:
    virtual void reset()
    {
        if (reset_called && !update_called)
            throw std::runtime_error("Multiple reset() calls detected without call to update()");

        reset_called = true;
    }
    virtual void update()
    {
        if (!reset_called)
            throw std::runtime_error("reset() must be called first before update()");
        update_called = true;
    }

    virtual ~UpdatableObject() = default;

    virtual void reportState(StateReporter& reporter)
    {
        unused(reporter);
        //default implementation doesn't do anything
    }

    virtual UpdatableObject* getPhysicsBody()
    {
        return nullptr;
    }

    virtual ClockBase* clock()
    {
        return ClockFactory::get();
    }


protected:
    void clearResetUpdateAsserts()
    {
        reset_called = false;
        update_called = false;
    }

private:
    bool reset_called = false;
    bool update_called = false;
};

}} //namespace
#endif

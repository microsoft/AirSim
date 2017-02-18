// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_UpdatableObject_hpp
#define airsim_core_UpdatableObject_hpp

#include "common/Common.hpp"
#include "StateReporter.hpp"

namespace msr {
namespace airlib {

class UpdatableObject {
  public:
    virtual void reset() = 0;
    virtual void update(real_T dt) = 0;
    virtual ~UpdatableObject() = default;

    virtual void reportState(StateReporter& reporter) {
        //default implementation doesn't do anything
    }

    virtual UpdatableObject* getPhysicsBody() {
        return nullptr;
    }

};

}
} //namespace
#endif

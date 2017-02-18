// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_ControllerBase_hpp
#define msr_air_copter_sim_ControllerBase_hpp

#include "common/UpdatableObject.hpp"

namespace msr {
namespace airlib {

class ControllerBase : public UpdatableObject {
  public:
    virtual void reset() = 0;
    virtual void update(real_T dt) = 0;
    //return 0 to 1 (corresponds to zero to full thrust)
    virtual real_T getRotorControlSignal(unsigned int rotor_index) = 0;
};

}
} //namespace
#endif

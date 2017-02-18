// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_SensorBase_hpp
#define msr_air_copter_sim_SensorBase_hpp

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "physics/PhysicsBody.hpp"


namespace msr {
namespace airlib {


class SensorBase : public UpdatableObject  {
  public:
    struct GroundTruth {
        const PhysicsBody* body;
        const Kinematics::State* kinematics;
        const Environment* environment;
    };


    //redeclare UpdatableState implementation ***//
    virtual void reset() override = 0;
    virtual void update(real_T dt) override = 0;


    const GroundTruth& getGroundTruth() const {
        return *ground_truth_;
    }

    virtual ~SensorBase() = default;

  protected:
    void initialize(GroundTruth* ground_truth) {
        ground_truth_ = ground_truth;
    }

  private:
    //ground truth can be shared between many sensors
    GroundTruth* ground_truth_;
};



}
} //namespace
#endif

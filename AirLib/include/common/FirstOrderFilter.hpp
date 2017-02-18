// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.


#ifndef airsimcore_firstorderfilter_hpp
#define airsimcore_firstorderfilter_hpp

#include <cmath>
#include "UpdatableObject.hpp"
#include "Common.hpp"

namespace msr {
namespace airlib {

template <typename T>
class FirstOrderFilter : UpdatableObject {
    /*
    This class can be used to apply a first order filter on a signal.
    It allows different acceleration and deceleration time constants.

    Short review of discrete time implementation of first order system:
    Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
    continuous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
    discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
    */
  private:
    float timeConstant_;
    T output_, input_;
    T initial_output_, initial_input_;
  public:
    FirstOrderFilter() {
        FirstOrderFilter::reset();
    }
    FirstOrderFilter(float timeConstant, T initial_input, T initial_output) {
        initialize(timeConstant, initial_input, initial_output);
    }
    void initialize(float timeConstant, T initial_input, T initial_output) {
        timeConstant_ = timeConstant;
        initial_input_ = initial_input;
        initial_output_ = initial_output;
        FirstOrderFilter::reset();
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override {
        input_ = initial_input_;
        output_ = initial_output_;
    }

    virtual void update(real_T dt) override {
        //lower the weight for previous value if its been long time
        //TODO: minimize use of exp
        float alpha = expf(-dt / timeConstant_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        output_ = output_ * alpha + input_ * (1 - alpha);
    }
    //*** End: UpdatableState implementation ***//


    void setInput(T input) {
        input_ = input;
    }
    T getInput() const {
        return input_;
    }

    T getOutput() const {
        return output_;
    }
};

}
} //namespace
#endif

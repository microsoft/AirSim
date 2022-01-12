#pragma once

#include <cmath>
#include "IUpdatable.hpp"
#include "CommonStructs.hpp"
#include "common/Common.hpp"

namespace simple_flight
{

class IEkf : public IUpdatable
{    
public:
    virtual bool checkEkfEnabled() const = 0;

    // getters
    virtual const VectorNXf& getEkfStates() const = 0;
    virtual const VectorMath::Vector17f& getEkfMeasurements() const = 0;
    virtual const MatrixNXxNXf& getEkfCovariance() const = 0;
    virtual const VectorMath::Matrix3x3f& getEkfEulerAnglesCovariance() const = 0;
};

} //namespace
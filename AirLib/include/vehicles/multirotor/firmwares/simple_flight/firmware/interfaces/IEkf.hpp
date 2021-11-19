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
    virtual void update() override
    {
        IUpdatable::update();
    }

    // setters
    void setEkfStates(VectorMath::Vector17f states)
    {
        states_ = states;
    }
    
    // getters
    const VectorMath::Vector17f& getEkfStates() const
    {
        return states_;
    }

    // setters
    void setEkfCovariance(VectorMath::Matrix17x17f covariance)
    {
        covariance_ = covariance;
    }
    
    // getters
    const VectorMath::Matrix17x17f& getEkfCovariance() const
    {
        return covariance_;
    }

protected:
    VectorMath::Vector17f states_;
    VectorMath::Matrix17x17f covariance_;
    VectorMath::Matrix13x13f Q_;
};

} //namespace
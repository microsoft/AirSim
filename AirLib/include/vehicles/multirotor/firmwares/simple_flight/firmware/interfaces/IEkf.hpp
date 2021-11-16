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
    void setEkfStates(VectorMath::EkfStates states)
    {
        states_ = states;
    }
    
    // getters
    const VectorMath::EkfStates& getEkfStates() const
    {
        return states_;
    }

protected:
    VectorMath::EkfStates states_;
};

} //namespace
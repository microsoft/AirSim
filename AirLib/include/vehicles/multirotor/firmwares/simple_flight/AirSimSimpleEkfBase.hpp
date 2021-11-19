// Liscence info

#ifndef msr_airlib_AirSimSimpleEkfBase_hpp
#define msr_airlib_AirSimSimpleEkfBase_hpp

#include <cmath>
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"
#include "firmware/interfaces/IEkf.hpp"

namespace msr
{
namespace airlib
{

class AirSimSimpleEkfBase : public simple_flight::IEkf
{    
protected:
    struct LastTimes
    {
        TTimePoint state_propagation;
        TTimePoint cov_propagation;
    };

protected:
    // EKF states
    // VectorMath::EkfStates states_;
    // EKF covariances
    // timestamps
    LastTimes last_times_;
};

}
} //namespace
#endif
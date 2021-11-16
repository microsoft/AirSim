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
    // EKF states
    // VectorMath::EkfStates states_;
    // EKF covariances
    VectorMath::EkfCovariance covariance_;
    // timestamps
    TTimePoint last_statePropagation_time_;
};

}
} //namespace
#endif
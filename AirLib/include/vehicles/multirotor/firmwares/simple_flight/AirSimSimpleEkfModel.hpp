// Liscence info

#ifndef msr_airlib_AirSimSimpleEkfModel_hpp
#define msr_airlib_AirSimSimpleEkfModel_hpp

#include <exception>
#include <vector>
#include "firmware/interfaces/IUpdatable.hpp"
#include "firmware/interfaces/IBoard.hpp"
#include "common/FrequencyLimiter.hpp"
#include "firmware/interfaces/IEkf.hpp"

// #include "firmware/Params.hpp"
// #include "common/Common.hpp"
// #include "common/ClockFactory.hpp"
// #include "physics/Kinematics.hpp"                                                                   

namespace msr
{
namespace airlib
{

    class AirSimSimpleEkfModel
    {
    protected:
        // ---------------------------------------------------------------------
        // Mathematical functions
        // ---------------------------------------------------------------------

        void f()
        {
            // the state_dot vector
        }

        void h_mag()
        {
            // 
        }

        void h_baro()
        {
            // 
        }

        void h_GPS()
        {
            // 
        }

        // ---------------------------------------------------------------------
        // Mathematical jacobians
        // ---------------------------------------------------------------------

        void df_dx()
        {
            // jacobian wrt the states vector
        }

        void df_dw()
        {
            // jacobian wrt the process noise vector
        }

        void dh_mag_dx()
        {
            // jacobian wrt the states vector
        }

        void dh_baro_dx()
        {
            // jacobian wrt the states vector
        }

        void dh_GPS_dx()
        {
            // jacobian wrt the states vector
        }

    };

}
} //namespace
#endif
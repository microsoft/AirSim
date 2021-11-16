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

constexpr float G_0 = 9.81f;      
constexpr float R_E = 6357000.0f;

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

        void evaluateStateDot(float f_local[10], float x[17], float u[6])
        {
            // compute x_dot = f_local
            /*
            x[0] = x
            x[1] = y
            x[2] = z
            x[3] = u
            x[4] = v
            x[5] = w
            x[6] = q0
            x[7] = q1
            x[8] = q2
            x[9] = q3
            x[10] = bias_accel_x
            x[11] = bias_accel_y
            x[12] = bias_accel_z
            x[13] = bias_gyro_x
            x[14] = bias_gyro_x
            x[15] = bias_gyro_x
            x[16] = bias_baro

            u[0] = accel_x
            u[1] = accel_y
            u[2] = accel_z
            u[3] = gyro_x
            u[4] = gyro_y
            u[5] = gyro_z
            */     

            // f_pos
            f_local[0] = x[3];
            f_local[1] = x[4];
            f_local[2] = x[5];
            // f_vel
            f_local[3] =    (x[6]*x[6] + x[7]*x[7] - x[8]*x[8] - x[9]*x[9])*    (u[0] + x[10])
                            + 2*(x[7]*x[8] - x[6]*x[9])*                        (u[1] + x[11])
                            + 2*(x[6]*x[8] + x[7]*x[9])*                        (u[2] + x[12]);
            f_local[4] =    2*(x[7]*x[8] + x[6]*x[9])*                          (u[0] + x[10])
                            + (x[6]*x[6] - x[7]*x[7] + x[8]*x[8] - x[9]*x[9])*  (u[1] + x[11])
                            + 2*(x[8]*x[9] - x[6]*x[7])*                        (u[2] + x[12]);
            f_local[5] =    2*(x[7]*x[9] - x[6]*x[8])*                          (u[0] + x[10])
                            + 2*(x[8]*x[9] + x[6]*x[7])*                        (u[1] + x[11])
                            + (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] + x[9]*x[9])*  (u[2] + x[12])
                            + G_0*(1 + 2*x[2]/R_E);
            // f_ori
            f_local[6] = 0.5f * (-x[7]*(u[3] + x[13]) - x[8]*(u[4] + x[14]) - x[9]*(u[5] + x[15]));
            f_local[7] = 0.5f * ( x[6]*(u[3] + x[13]) - x[9]*(u[4] + x[14]) - x[8]*(u[5] + x[15]));
            f_local[8] = 0.5f * ( x[9]*(u[3] + x[13]) + x[6]*(u[4] + x[14]) - x[7]*(u[5] + x[15]));
            f_local[9] = 0.5f * (-x[8]*(u[3] + x[13]) + x[7]*(u[4] + x[14]) + x[6]*(u[5] + x[15]));

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

        void df_dx(float A[17][17], float x[17], float u[6])
        {
            // compute jacobian wrt the states vector df_dx = A
            /*
            x[0] = x
            x[1] = y
            x[2] = z
            x[3] = u
            x[4] = v
            x[5] = w
            x[6] = q0
            x[7] = q1
            x[8] = q2
            x[9] = q3
            x[10] = bias_accel_x
            x[11] = bias_accel_y
            x[12] = bias_accel_z
            x[13] = bias_gyro_x
            x[14] = bias_gyro_x
            x[15] = bias_gyro_x
            x[16] = bias_baro

            u[0] = accel_x
            u[1] = accel_y
            u[2] = accel_z
            u[3] = gyro_x
            u[4] = gyro_y
            u[5] = gyro_z
            */  

            // set all elements to zero
            float A[17][17];
            memset(&A[0][0], 0, sizeof(A));

            // df_pos_dx [0 1 2]
            A[0][3] = 1.0f;
            A[1][4] = 1.0f;
            A[2][5] = 1.0f;
            // df_vel_dx [3 4 5]
            A[5][2] = 2*G_0/R_E;
            // df_vel_q0 [3 4 5]
            A[3][6] = 2*x[6](u[0] + x[10]) - 2*x[6](u[0] + x[10]) + 2*x[6](u[0] + x[10])



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
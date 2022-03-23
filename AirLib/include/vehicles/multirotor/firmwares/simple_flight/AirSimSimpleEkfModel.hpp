// Liscence info

#ifndef msr_airlib_AirSimSimpleEkfModel_hpp
#define msr_airlib_AirSimSimpleEkfModel_hpp

#include <exception>
#include <vector>
#include "firmware/interfaces/IUpdatable.hpp"
#include "firmware/interfaces/IBoard.hpp"

constexpr float G_0 = EarthUtils::Gravity; // for less than 10000m const gravity is taken
constexpr float R_E = EARTH_RADIUS;

namespace msr
{
namespace airlib
{

    class AirSimSimpleEkfModel
    {
    public:
        // ---------------------------------------------------------------------
        // Mathematical functions
        // ---------------------------------------------------------------------

        static void insPositionKinematics(float position_dot[3], float lin_velocity[3])
        {
            float u = lin_velocity[0];
            float v = lin_velocity[1];
            float w = lin_velocity[2];

            // position_dot
            position_dot[0] = u; // x_dot
            position_dot[1] = v; // y_dot
            position_dot[2] = w; // z_dot
        }

        static void insVelocityKinematics(float lin_velocity_dot[3], float quaternions[4], float specific_forces[3], float accel_biases[3])
        {
            float q0 = quaternions[0];
            float q1 = quaternions[1];
            float q2 = quaternions[2];
            float q3 = quaternions[3];

            float f_x = specific_forces[0];
            float f_y = specific_forces[1];
            float f_z = specific_forces[2];

            float b_f_x = accel_biases[0];
            float b_f_y = accel_biases[1];
            float b_f_z = accel_biases[2];

            // velocity_dot. Transform specific forces from B frame to O frame and evaluate velocity_dot
            lin_velocity_dot[0] = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (f_x - b_f_x) + 2.0f * (q1 * q2 - q0 * q3) * (f_y - b_f_y) + 2.0f * (q0 * q2 + q1 * q3) * (f_z - b_f_z);
            lin_velocity_dot[1] = 2.0f * (q1 * q2 + q0 * q3) * (f_x - b_f_x) + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * (f_y - b_f_y) + 2.0f * (q2 * q3 - q0 * q1) * (f_z - b_f_z);
            lin_velocity_dot[2] = 2.0f * (q1 * q3 - q0 * q2) * (f_x - b_f_x) + 2.0f * (q2 * q3 + q0 * q1) * (f_y - b_f_y) + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * (f_z - b_f_z)
                                  //+ G_0*(1 + 2.0f*x[2]/R_E);
                                  + G_0; //+0.001f;
        }

        static void insAttitudeKinematics(float attitude_dot[4], float quaternions[4], float gyro_rates[3], float gyro_biases[3])
        {
            float q0 = quaternions[0];
            float q1 = quaternions[1];
            float q2 = quaternions[2];
            float q3 = quaternions[3];

            float omega_x = gyro_rates[0];
            float omega_y = gyro_rates[1];
            float omega_z = gyro_rates[2];

            float b_omega_x = gyro_biases[0];
            float b_omega_y = gyro_biases[1];
            float b_omega_z = gyro_biases[2];

            float lambda = 1.0f - (q0 * q0 + q1 * q1 + q3 * q2 + q3 * q3);
            float k = 300.0f; // k*h <= 1, h = 0.003

            // attitude_dot. Transform specific forces from B frame to O frame and evaluate velocity_dot
            attitude_dot[0] = 0.5f * (-q1 * (omega_x - b_omega_x) - q2 * (omega_y - b_omega_y) - q3 * (omega_z - b_omega_z) + q0 * 2 * k * lambda);
            attitude_dot[1] = 0.5f * (q0 * (omega_x - b_omega_x) - q3 * (omega_y - b_omega_y) + q2 * (omega_z - b_omega_z) + q1 * 2 * k * lambda);
            attitude_dot[2] = 0.5f * (q3 * (omega_x - b_omega_x) + q0 * (omega_y - b_omega_y) - q1 * (omega_z - b_omega_z) + q2 * 2 * k * lambda);
            attitude_dot[3] = 0.5f * (-q2 * (omega_x - b_omega_x) + q1 * (omega_y - b_omega_y) + q0 * (omega_z - b_omega_z) + q3 * 2 * k * lambda);
        }

        static void evaluateStateDot(float x_dot[simple_flight::NX], float x[simple_flight::NX], float u[simple_flight::NU])
        {
            float lin_velocity[3];
            lin_velocity[0] = x[3];
            lin_velocity[1] = x[4];
            lin_velocity[2] = x[5];

            // position kinematics
            float position_dot[3];
            insPositionKinematics(position_dot, lin_velocity);

            float quaternions[4];
            quaternions[0] = x[6];
            quaternions[1] = x[7];
            quaternions[2] = x[8];
            quaternions[3] = x[9];

            float specific_forces[3];
            specific_forces[0] = u[0];
            specific_forces[1] = u[1];
            specific_forces[2] = u[2];

            float accel_biases[3];
            accel_biases[0] = x[10];
            accel_biases[1] = x[11];
            accel_biases[2] = x[12];

            // linear velocity kinematics
            float lin_velocity_dot[3];
            insVelocityKinematics(lin_velocity_dot, quaternions, specific_forces, accel_biases);

            float gyro_rates[3];
            gyro_rates[0] = u[3];
            gyro_rates[1] = u[4];
            gyro_rates[2] = u[5];

            float gyro_biases[3];
            gyro_biases[0] = x[13];
            gyro_biases[1] = x[14];
            gyro_biases[2] = x[15];

            // attitude kinematics
            float quaternions_dot[4];
            insAttitudeKinematics(quaternions_dot, quaternions, gyro_rates, gyro_biases);

            x_dot[0] = position_dot[0];
            x_dot[1] = position_dot[1];
            x_dot[2] = position_dot[2];
            x_dot[3] = lin_velocity_dot[0];
            x_dot[4] = lin_velocity_dot[1];
            x_dot[5] = lin_velocity_dot[2];
            x_dot[6] = quaternions_dot[0];
            x_dot[7] = quaternions_dot[1];
            x_dot[8] = quaternions_dot[2];
            x_dot[9] = quaternions_dot[3];
            x_dot[10] = 0.0f;
            x_dot[11] = 0.0f;
            x_dot[12] = 0.0f;
            x_dot[13] = 0.0f;
            x_dot[14] = 0.0f;
            x_dot[15] = 0.0f;
            x_dot[16] = 0.0f;
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

        static void evaluateA(simple_flight::MatrixNXxNXf* A, float x[simple_flight::NX], float u[simple_flight::NU])
        {
            *A = simple_flight::MatrixNXxNXf::Zero();

            float q0 = x[6]; // quaternions
            float q1 = x[7]; // quaternions
            float q2 = x[8]; // quaternions
            float q3 = x[9]; // quaternions
            float b_f_x = x[10]; // specific forces
            float b_f_y = x[11]; // specific forces
            float b_f_z = x[12]; // specific forces
            float b_omega_x = x[13]; // gyro_biases
            float b_omega_y = x[14]; // gyro_biases
            float b_omega_z = x[15]; // gyro_biases

            float f_x = u[0]; // specific forces
            float f_y = u[1]; // specific forces
            float f_z = u[2]; // specific forces
            float omega_x = u[3]; // gyro rates
            float omega_y = u[4]; // gyro rates
            float omega_z = u[5]; // gyro rates

            float k = 300.0f;

            // df_pos_dx [0 1 2][:]
            (*A)(0, 3) = 1.0f;
            (*A)(1, 4) = 1.0f;
            (*A)(2, 5) = 1.0f;
            // df_vel_dx [3 4 5][:]
            //(*A)(5, 2) = 2.0f*G_0/R_E;
            // df_vel_q0 [3 4 5][6]
            (*A)(3, 6) = 2.0f * q0 * (f_x - b_f_x) - 2.0f * q3 * (f_y - b_f_y) + 2.0f * q2 * (f_z - b_f_z);
            (*A)(4, 6) = 2.0f * q3 * (f_x - b_f_x) + 2.0f * q0 * (f_y - b_f_y) - 2.0f * q1 * (f_z - b_f_z);
            (*A)(5, 6) = -2.0f * q2 * (f_x - b_f_x) + 2.0f * q1 * (f_y - b_f_y) + 2.0f * q0 * (f_z - b_f_z);
            // df_vel_q1 [3 4 5][7]
            (*A)(3, 7) = 2.0f * q1 * (f_x - b_f_x) + 2.0f * q2 * (f_y - b_f_y) + 2.0f * q3 * (f_z - b_f_z);
            (*A)(4, 7) = 2.0f * q2 * (f_x - b_f_x) - 2.0f * q1 * (f_y - b_f_y) - 2.0f * q0 * (f_z - b_f_z);
            (*A)(5, 7) = 2.0f * q3 * (f_x - b_f_x) + 2.0f * q0 * (f_y - b_f_y) - 2.0f * q1 * (f_z - b_f_z);
            // df_vel_q2 [3 4 5][8]
            (*A)(3, 8) = -2.0f * q2 * (f_x - b_f_x) + 2.0f * q1 * (f_y - b_f_y) + 2.0f * q0 * (f_z - b_f_z);
            (*A)(4, 8) = 2.0f * q1 * (f_x - b_f_x) + 2.0f * q2 * (f_y - b_f_y) + 2.0f * q3 * (f_z - b_f_z);
            (*A)(5, 8) = -2.0f * q0 * (f_x - b_f_x) + 2.0f * q3 * (f_y - b_f_y) - 2.0f * q2 * (f_z - b_f_z);
            // df_vel_q3 [3 4 5][9]
            (*A)(3, 9) = -2.0f * q3 * (f_x - b_f_x) - 2.0f * q0 * (f_y - b_f_y) + 2.0f * q1 * (f_z - b_f_z);
            (*A)(4, 9) = 2.0f * q0 * (f_x - b_f_x) - 2.0f * q3 * (f_y - b_f_y) + 2.0f * q2 * (f_z - b_f_z);
            (*A)(5, 9) = 2.0f * q1 * (f_x - b_f_x) + 2.0f * q2 * (f_y - b_f_y) + 2.0f * q3 * (f_z - b_f_z);
            // df_vel_xbias [3 4 5][10 11 12 ...]
            (*A)(3, 10) = -(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            (*A)(3, 11) = -2.0f * (q1 * q2 - q0 * q3);
            (*A)(3, 12) = -2.0f * (q0 * q2 + q1 * q3);
            (*A)(4, 10) = -2.0f * (q1 * q2 + q0 * q3);
            (*A)(4, 11) = -(q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
            (*A)(4, 12) = -2.0f * (q2 * q3 - q0 * q1);
            (*A)(5, 10) = -2.0f * (q1 * q3 - q0 * q2);
            (*A)(5, 11) = -2.0f * (q2 * q3 + q0 * q1);
            (*A)(5, 12) = -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            // df_ori_[q0 q1 q2 q3] [6 7 8 9][6 7 8 9]
            (*A)(6, 6) = 0.5f * (2 * k - 6 * k * q0 * q0 - 2 * k * q1 * q1 - 2 * k * q2 * q2 - 2 * k * q3 * q3);
            (*A)(6, 7) = 0.5f * (-(omega_x - b_omega_x) - 4 * k * q0 * q1);
            (*A)(6, 8) = 0.5f * (-(omega_y - b_omega_y) - 4 * k * q0 * q2);
            (*A)(6, 9) = 0.5f * (-(omega_z - b_omega_z) - 4 * k * q0 * q3);
            (*A)(7, 6) = 0.5f * (omega_x - b_omega_x - 4 * k * q1 * q0);
            (*A)(7, 7) = 0.5f * (2 * k - 2 * k * q0 * q0 - 6 * k * q1 * q1 - 2 * k * q2 * q2 - 2 * k * q3 * q3);
            (*A)(7, 8) = 0.5f * (omega_z - b_omega_z - 4 * k * q1 * q2);
            (*A)(7, 9) = 0.5f * (-(omega_y - b_omega_y) - 4 * k * q1 * q3);
            (*A)(8, 6) = 0.5f * (omega_y - b_omega_y - 4 * k * q2 * q0);
            (*A)(8, 7) = 0.5f * (-(omega_z - b_omega_z) - 4 * k * q2 * q1);
            (*A)(8, 8) = 0.5f * (2 * k - 2 * k * q0 * q0 - 2 * k * q1 * q1 - 6 * k * q2 * q2 - 2 * k * q3 * q3);
            (*A)(8, 9) = 0.5f * (omega_x - b_omega_x - 4 * k * q2 * q3);
            (*A)(9, 6) = 0.5f * (omega_z - b_omega_z - 4 * k * q3 * q0);
            (*A)(9, 7) = 0.5f * (omega_y - b_omega_y - 4 * k * q3 * q1);
            (*A)(9, 8) = 0.5f * (-(omega_x - b_omega_x) - 4 * k * q3 * q2);
            (*A)(9, 9) = 0.5f * (2 * k - 2 * k * q0 * q0 - 2 * k * q1 * q1 - 2 * k * q2 * q2 - 6 * k * q3 * q3);
            // df_ori_x_bias [6 7 8 9][10 11 12 13 14 15 16 17]
            (*A)(6, 13) = 0.5f * q1;
            (*A)(6, 14) = 0.5f * q2;
            (*A)(6, 15) = 0.5f * q3;
            (*A)(7, 13) = -0.5f * q0;
            (*A)(7, 14) = 0.5f * q3;
            (*A)(7, 15) = -0.5f * q2;
            (*A)(8, 13) = -0.5f * q3;
            (*A)(8, 14) = -0.5f * q0;
            (*A)(8, 15) = 0.5f * q1;
            (*A)(9, 13) = 0.5f * q2;
            (*A)(9, 14) = -0.5f * q1;
            (*A)(9, 15) = -0.5f * q0;
        }

        static void evaluateFiniteDifferenceA(simple_flight::MatrixNXxNXf* A, float x[simple_flight::NX], float u[simple_flight::NU])
        {
            *A = simple_flight::MatrixNXxNXf::Zero();

            float derivative_perturbation = 1E-5f;
            float x_plus[simple_flight::NX];
            float x_minus[simple_flight::NX];
            float f_plus[simple_flight::NX];
            float f_minus[simple_flight::NX];
            float df_dx_i_column[simple_flight::NX];

            for (int i = 0; i < simple_flight::NX; i++) {
                for (int j = 0; j < simple_flight::NX; j++) {
                    x_plus[j] = x[j];
                    x_minus[j] = x[j];
                }
                auto perturbation_i = std::max(1.0f, std::abs(x[i])) * derivative_perturbation;
                x_plus[i] = x[i] + perturbation_i;
                x_minus[i] = x[i] - perturbation_i;

                evaluateStateDot(f_plus, x_plus, u);
                evaluateStateDot(f_minus, x_minus, u);

                for (int j = 0; j < simple_flight::NX; j++) {
                    df_dx_i_column[j] = (f_plus[j] - f_minus[j]) / (2 * perturbation_i);
                }

                for (int j = 0; j < simple_flight::NX; j++) {
                    (*A)(j, i) = df_dx_i_column[j];
                }
            }
        }

        static bool checkA(VectorMath::Matrix17x17f* A_error, VectorMath::Matrix17x17f* A, VectorMath::Matrix17x17f* A_finite, float row[17], float column[17])
        {
            *A_error = VectorMath::Matrix17x17f::Zero();

            float derivative_test_tolerance = 0.01f;

            for (int i = 0; i < 17; i++) {
                for (int j = 0; j < 17; j++) {
                    float a = (*A)(i, j);
                    float a_finite = (*A_finite)(i, j);
                    (*A_error)(i, j) = std::abs(a - a_finite) / std::max(std::abs(a_finite), derivative_test_tolerance);
                }
            }
            int k = 0;
            for (int i = 0; i < 17; i++) {
                row[i] = -1;
            }
            for (int i = 0; i < 17; i++) {
                column[i] = -1;
            }
            for (int i = 0; i < 17; i++) {
                for (int j = 0; j < 17; j++) {
                    if ((*A_error)(i, j) > derivative_test_tolerance) {
                        row[k] = static_cast<float>(i);
                        column[k] = static_cast<float>(j);
                        k++;
                    }
                }
            }
            for (int i = 0; i < 17; i++) {
                for (int j = 0; j < 17; j++) {
                    if ((*A_error)(i, j) > derivative_test_tolerance) {
                        return false;
                    }
                }
            }
            return true;
        }

        static void evaluateB_w(simple_flight::MatrixNXxNWf* B_w, float x[simple_flight::NX], float u[simple_flight::NU])
        {
            // set all elements to zero
            *B_w = simple_flight::MatrixNXxNWf::Zero();

            float q0 = x[6]; // quaternions
            float q1 = x[7]; // quaternions
            float q2 = x[8]; // quaternions
            float q3 = x[9]; // quaternions

            // df_vel_w [3 4 5][0 1 2 ...]
            (*B_w)(3, 0) = -(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            (*B_w)(3, 1) = -2.0f * (q1 * q2 - q0 * q3);
            (*B_w)(3, 2) = -2.0f * (q0 * q2 + q1 * q3);
            (*B_w)(4, 0) = -2.0f * (q1 * q2 + q0 * q3);
            (*B_w)(4, 1) = -(q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
            (*B_w)(4, 2) = -2.0f * (q2 * q3 - q0 * q1);
            (*B_w)(5, 0) = -2.0f * (q1 * q3 - q0 * q2);
            (*B_w)(5, 1) = -2.0f * (q2 * q3 + q0 * q1);
            (*B_w)(5, 2) = -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            // df_ori_x_bias [6 7 8 9][10 11 12 13 14 15 16 17]
            (*B_w)(6, 3) = 0.5f * q1;
            (*B_w)(6, 4) = 0.5f * q2;
            (*B_w)(6, 5) = 0.5f * q3;
            (*B_w)(7, 3) = -0.5f * q0;
            (*B_w)(7, 4) = 0.5f * q3;
            (*B_w)(7, 5) = -0.5f * q2;
            (*B_w)(8, 3) = -0.5f * q3;
            (*B_w)(8, 4) = -0.5f * q0;
            (*B_w)(8, 5) = 0.5f * q1;
            (*B_w)(9, 3) = 0.5f * q2;
            (*B_w)(9, 4) = -0.5f * q1;
            (*B_w)(9, 5) = -0.5f * q0;
            // df_bias_w
            (*B_w)(10, 6) = 1.0f;
            (*B_w)(11, 7) = 1.0f;
            (*B_w)(12, 8) = 1.0f;
            (*B_w)(13, 9) = 1.0f;
            (*B_w)(14, 10) = 1.0f;
            (*B_w)(15, 11) = 1.0f;
            (*B_w)(16, 12) = 1.0f;
        }

        static void dh_mag_dx()
        {
            // jacobian wrt the states vector
        }

        static void evaluateCBaro(simple_flight::Matrix1xNXf* C_baro)
        {
            // dh_baro_dx jacobian wrt the states vector
            *C_baro = simple_flight::Matrix1xNXf::Zero();
            (*C_baro)(2) = -1.0f;
            (*C_baro)(16) = -1.0f;
        }

        static void evaluateCGps(simple_flight::Matrix6xNXf* C_gps)
        {
            // dh_gps_dx jacobian wrt the states vector
            *C_gps = simple_flight::Matrix6xNXf::Zero();
            (*C_gps)(0, 0) = 1.0f;
            (*C_gps)(1, 1) = 1.0f;
            (*C_gps)(2, 2) = 1.0f;
            (*C_gps)(3, 3) = 1.0f;
            (*C_gps)(4, 4) = 1.0f;
            (*C_gps)(5, 5) = 1.0f;
        }

        static void evaluatehMag(float h_mag[3], float x[17], float earth_mag[3])
        {
            float q0 = x[6];
            float q1 = x[7];
            float q2 = x[8];
            float q3 = x[9];

            float tau_x = earth_mag[0];
            float tau_y = earth_mag[1];
            float tau_z = earth_mag[2];

            h_mag[0] = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * tau_x + (2.0f * (q1 * q2 + q0 * q3)) * tau_y + (2.0f * (q1 * q3 - q0 * q2)) * tau_z;
            h_mag[1] = (2.0f * (q1 * q2 - q0 * q3)) * tau_x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * tau_y + (2.0f * (q2 * q3 + q0 * q1)) * tau_z;
            h_mag[2] = (2.0f * (q0 * q2 + q1 * q3)) * tau_x + (2.0f * (q2 * q3 - q0 * q1)) * tau_y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * tau_z;
        }

        static void evaluateCMag(simple_flight::Matrix3xNXf* C_mag, float x[17], float earth_mag[3])
        {
            // dh_mag_dx jacobian wrt the states vector
            *C_mag = simple_flight::Matrix3xNXf::Zero();

            float q0 = x[6];
            float q1 = x[7];
            float q2 = x[8];
            float q3 = x[9];

            float tau_x = earth_mag[0];
            float tau_y = earth_mag[1];
            float tau_z = earth_mag[2];

            // dh_mag_q0
            (*C_mag)(0, 6) = 2.0f * q0 * tau_x + 2.0f * q3 * tau_y - 2.0f * q2 * tau_z;
            (*C_mag)(1, 6) = -2.0f * q3 * tau_x + 2.0f * q0 * tau_y + 2.0f * q1 * tau_z;
            (*C_mag)(2, 6) = 2.0f * q2 * tau_x - 2.0f * q1 * tau_y + 2.0f * q0 * tau_z;
            // dh_mag_q1
            (*C_mag)(0, 7) = 2.0f * q1 * tau_x + 2.0f * q2 * tau_y + 2.0f * q3 * tau_z;
            (*C_mag)(1, 7) = 2.0f * q2 * tau_x - 2.0f * q1 * tau_y + 2.0f * q0 * tau_z;
            (*C_mag)(2, 7) = 2.0f * q3 * tau_x - 2.0f * q0 * tau_y - 2.0f * q1 * tau_z;
            // dh_mag_q2
            (*C_mag)(0, 8) = -2.0f * q2 * tau_x + 2.0f * q1 * tau_y - 2.0f * q0 * tau_z;
            (*C_mag)(1, 8) = 2.0f * q1 * tau_x + 2.0f * q2 * tau_y + 2.0f * q3 * tau_z;
            (*C_mag)(2, 8) = 2.0f * q0 * tau_x + 2.0f * q3 * tau_y - 2.0f * q2 * tau_z;
            // dh_mag_q3
            (*C_mag)(0, 9) = -2.0f * q3 * tau_x + 2.0f * q0 * tau_y + 2.0f * q1 * tau_z;
            (*C_mag)(1, 9) = -2.0f * q0 * tau_x - 2.0f * q3 * tau_y + 2.0f * q2 * tau_z;
            (*C_mag)(2, 9) = 2.0f * q1 * tau_x + 2.0f * q2 * tau_y + 2.0f * q3 * tau_z;
        }

        static void evaluateCPseudo(simple_flight::Matrix1xNXf* C_pseudo, float x[17])
        {
            // dh_gps_dx jacobian wrt the states vector
            *C_pseudo = simple_flight::Matrix1xNXf::Zero();

            float q0 = x[6];
            float q1 = x[7];
            float q2 = x[8];
            float q3 = x[9];

            // dh_mag_dq
            (*C_pseudo)(6) = 2.0f * q0;
            (*C_pseudo)(7) = 2.0f * q1;
            (*C_pseudo)(8) = 2.0f * q2;
            (*C_pseudo)(9) = 2.0f * q3;
        }

        static void evaluateCEulerAngles(VectorMath::Matrix3x4f* C_euler, float x[17])
        {
            // dattitude_dq jacobian of euler angles wrt the quaternions
            *C_euler = VectorMath::Matrix3x4f::Zero();

            float q0 = x[6];
            float q1 = x[7];
            float q2 = x[8];
            float q3 = x[9];

            // d_phi_dq
            float term_1 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
            float term_2 = q2 * q3 + q0 * q1;
            float denominator = term_1 * term_1 + 4 * term_2 * term_2;
            float dphi_q0 = (2 * q1 * term_1 - 4 * q0 * term_2) / denominator;
            float dphi_q1 = (2 * q0 * term_1 + 4 * q1 * term_2) / denominator;
            float dphi_q2 = (2 * q3 * term_1 + 4 * q2 * term_2) / denominator;
            float dphi_q3 = (2 * q2 * term_1 - 4 * q3 * term_2) / denominator;

            // d_theta_dq
            term_1 = q1 * q3 - q0 * q2;
            denominator = sqrt(1 - 4 * term_1 * term_1);
            float dtheta_q0 = 2 * q2 / denominator;
            float dtheta_q1 = -2 * q3 / denominator;
            float dtheta_q2 = 2 * q0 / denominator;
            float dtheta_q3 = -2 * q1 / denominator;

            // d_psi_dq
            term_1 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
            term_2 = q1 * q2 + q0 * q3;
            denominator = term_1 * term_1 + 4 * term_2 * term_2;
            float dpsi_q0 = (2 * q3 * term_1 - 4 * q0 * term_2) / denominator;
            float dpsi_q1 = (2 * q2 * term_1 - 4 * q1 * term_2) / denominator;
            float dpsi_q2 = (2 * q1 * term_1 + 4 * q2 * term_2) / denominator;
            float dpsi_q3 = (2 * q0 * term_1 + 4 * q3 * term_2) / denominator;

            (*C_euler)(0, 0) = dphi_q0;
            (*C_euler)(0, 1) = dphi_q1;
            (*C_euler)(0, 2) = dphi_q2;
            (*C_euler)(0, 3) = dphi_q3;
            (*C_euler)(1, 0) = dtheta_q0;
            (*C_euler)(1, 1) = dtheta_q1;
            (*C_euler)(1, 2) = dtheta_q2;
            (*C_euler)(1, 3) = dtheta_q3;
            (*C_euler)(2, 0) = dpsi_q0;
            (*C_euler)(2, 1) = dpsi_q1;
            (*C_euler)(2, 2) = dpsi_q2;
            (*C_euler)(2, 3) = dpsi_q3;
        }

        static void rungeKutta(float xp[17], float x[17], float u[6], float dt, int size = 17)
        {
            float k1[17] = { 0 };
            float k2[17] = { 0 };
            float k3[17] = { 0 };
            float k4[17] = { 0 };
            float x_dot[17] = { 0 };
            float x_temp[17] = { 0 };

            evaluateStateDot(x_dot, x, u);
            for (int n = 0; n < size; n++) {
                k1[n] = x_dot[n] * dt;
                x_temp[n] = x[n] + k1[n] / 2;
            }
            evaluateStateDot(x_dot, x_temp, u);
            for (int n = 0; n < size; n++) {
                k2[n] = x_dot[n] * dt;
                x_temp[n] = x[n] + k2[n] / 2;
            }
            evaluateStateDot(x_dot, x_temp, u);
            for (int n = 0; n < size; n++) {
                k3[n] = x_dot[n] * dt;
                x_temp[n] = x[n] + k3[n];
            }
            evaluateStateDot(x_dot, x_temp, u);
            for (int n = 0; n < size; n++) {
                k4[n] = x_dot[n] * dt;
                xp[n] = x[n] + k1[n] / 6 + k2[n] / 3 + k3[n] / 4 + k4[n] / 6;
            }
        }

        static void heun(float xp[17], float x[17], float u[6], float up[6], float dt, int size = 17)
        {
            float k1[17] = { 0 };
            float k2[17] = { 0 };
            float x_dot[17] = { 0 };
            float x_temp[17] = { 0 };

            evaluateStateDot(x_dot, x, u);
            for (int n = 0; n < size; n++) {
                k1[n] = x_dot[n] * dt;
                x_temp[n] = x[n] + k1[n];
            }
            evaluateStateDot(x_dot, x_temp, up);
            for (int n = 0; n < size; n++) {
                k2[n] = x_dot[n] * dt;
                xp[n] = x[n] + k1[n] / 2 + k2[n] / 2;
            }
        }

        static void inertialNavigation(float x_predicted[simple_flight::NX],
                                       float x[simple_flight::NX],
                                       float u[simple_flight::NU],
                                       float up[simple_flight::NU],
                                       float ang_acc[3],
                                       float dt_real)
        {
            float curr_quaternions[4];
            curr_quaternions[0] = x[6];
            curr_quaternions[1] = x[7];
            curr_quaternions[2] = x[8];
            curr_quaternions[3] = x[9];

            float curr_gyro_biases[3];
            curr_gyro_biases[0] = x[13];
            curr_gyro_biases[1] = x[14];
            curr_gyro_biases[2] = x[15];

            float curr_gyro_rates[3];
            // curr_gyro_rates[0] = u[3];
            // curr_gyro_rates[1] = u[4];
            // curr_gyro_rates[2] = u[5];
            curr_gyro_rates[0] = u[3] + 0.5f * dt_real * ang_acc[0];
            curr_gyro_rates[1] = u[4] + 0.5f * dt_real * ang_acc[1];
            curr_gyro_rates[2] = u[5] + 0.5f * dt_real * ang_acc[2];

            // attitude propagation
            float curr_attitude_dot[4];
            insAttitudeKinematics(curr_attitude_dot, curr_quaternions, curr_gyro_rates, curr_gyro_biases);
            float next_quaternions[4];
            float next_quaternions_minus[4];
            for (int i = 0; i < 4; i++) {
                next_quaternions_minus[i] = curr_quaternions[i] + dt_real * (curr_attitude_dot[i]);
            }
            float next_gyro_rates[3];
            next_gyro_rates[0] = up[3];
            next_gyro_rates[1] = up[4];
            next_gyro_rates[2] = up[5];
            float next_attitude_dot[4];
            // insAttitudeKinematics(curr_attitude_dot, curr_quaternions, next_gyro_rates, curr_gyro_biases);
            insAttitudeKinematics(next_attitude_dot, next_quaternions_minus, next_gyro_rates, curr_gyro_biases);
            // // for (int i=0; i<4; i++){
            // //     next_quaternions[i] = curr_quaternions[i] + dt_real*(attitude_dot[i] + attitude_dot_next[i])/2;
            // // }
            for (int i = 0; i < 4; i++) {
                // next_quaternions[i] = curr_quaternions[i] + dt_real*0.5f*(curr_attitude_dot[i] + next_attitude_dot[i]);
                next_quaternions[i] = curr_quaternions[i] + dt_real * (curr_attitude_dot[i]);
                // next_quaternions[i] = curr_quaternions[i] + dt_real*curr_attitude_dot[i]
                //                                           + dt_real*0.5f*(next_attitude_dot[i] - curr_attitude_dot[i]);
            }

            float curr_lin_velocity[3];
            curr_lin_velocity[0] = x[3];
            curr_lin_velocity[1] = x[4];
            curr_lin_velocity[2] = x[5];

            float curr_accel_biases[3];
            curr_accel_biases[0] = x[10];
            curr_accel_biases[1] = x[11];
            curr_accel_biases[2] = x[12];

            float curr_specific_forces[3];
            curr_specific_forces[0] = u[0];
            curr_specific_forces[1] = u[1];
            curr_specific_forces[2] = u[2];

            // velocity propagation
            float curr_lin_velocity_dot[3];
            insVelocityKinematics(curr_lin_velocity_dot, curr_quaternions, curr_specific_forces, curr_accel_biases);
            float next_lin_velocity_minus[3];
            float next_lin_velocity[3];
            for (int i = 0; i < 3; i++) {
                next_lin_velocity_minus[i] = curr_lin_velocity[i] + dt_real * (curr_lin_velocity_dot[i]);
            }
            float next_specific_forces[3];
            next_specific_forces[0] = up[0];
            next_specific_forces[1] = up[1];
            next_specific_forces[2] = up[2];
            float next_lin_velocity_dot[3];
            insVelocityKinematics(next_lin_velocity_dot, next_quaternions, next_specific_forces, curr_accel_biases);
            for (int i = 0; i < 3; i++) {
                next_lin_velocity[i] = curr_lin_velocity[i] + dt_real * 0.5f * (curr_lin_velocity_dot[i] + next_lin_velocity_dot[i]);
            }

            float curr_position[3];
            curr_position[0] = x[0];
            curr_position[1] = x[1];
            curr_position[2] = x[2];

            // position propagation
            float next_position[3];
            // float position_dot[3];
            // float position_dot_next[3];
            // insPositionKinematics(position_dot, next_lin_velocity_minus);
            // float next_position_minus[3];
            // for (int i=0; i<3; i++){
            //     next_position_minus[i] = curr_position[i] + dt_real*(position_dot[i]);
            // }
            // insPositionKinematics(position_dot_next, next_lin_velocity);
            // for (int i=0; i<3; i++){
            //     next_position[i] = curr_position[i] + dt_real*(position_dot[i] + position_dot_next[i])/2;
            // }
            for (int i = 0; i < 3; i++) {
                // next_position[i] = curr_position[i] + dt_real*0.5f*(0.5f*(curr_lin_velocity[i] + next_lin_velocity[i])
                //                                         + 0.25f*dt_real*(curr_lin_velocity_dot[i] + next_lin_velocity_dot[i]));
                next_position[i] = curr_position[i] + dt_real * curr_lin_velocity[i] + 0.5f * dt_real * dt_real * curr_lin_velocity_dot[i];
            }

            float curr_baro_bias = x[16];

            x_predicted[0] = next_position[0];
            x_predicted[1] = next_position[1];
            x_predicted[2] = next_position[2];
            x_predicted[3] = next_lin_velocity[0];
            x_predicted[4] = next_lin_velocity[1];
            x_predicted[5] = next_lin_velocity[2];
            x_predicted[6] = next_quaternions[0];
            x_predicted[7] = next_quaternions[1];
            x_predicted[8] = next_quaternions[2];
            x_predicted[9] = next_quaternions[3];

            x_predicted[10] = curr_accel_biases[0];
            x_predicted[11] = curr_accel_biases[1];
            x_predicted[12] = curr_accel_biases[2];
            x_predicted[13] = curr_gyro_biases[0];
            x_predicted[14] = curr_gyro_biases[1];
            x_predicted[15] = curr_gyro_biases[2];
            x_predicted[16] = curr_baro_bias;
        }
    };

}
} //namespace
#endif
// Liscence info

#ifndef msr_airlib_AirSimSimpleEkf_hpp
#define msr_airlib_AirSimSimpleEkf_hpp

#include <exception>
#include <vector>
#include <cmath>
#include "firmware/interfaces/IBoard.hpp"
#include "firmware/interfaces/ICommLink.hpp"
#include "common/FrequencyLimiter.hpp"
#include "AirSimSimpleEkfBase.hpp"
#include "AirSimSimpleEkfModel.hpp"
#include "AirSimSimpleEkfParams.hpp"
#include "common/GeodeticConverter.hpp"       

#define EKF_GROUND_TRUTH_MEAS_DIRECTIVE 0
#define EKF_BARO_DIRECTIVE 1
#define EKF_MAGNETO_DIRECTIVE 1
#define EKF_GPS_DIRECTIVE 1
#define EKF_PSEUDOMEAS_DIRECTIVE 1

namespace msr
{
namespace airlib
{

    class AirSimSimpleEkf : public AirSimSimpleEkfBase
        , private AirSimSimpleEkfModel
    {
    public:
        // Constructor
        AirSimSimpleEkf(const simple_flight::IBoard* board, simple_flight::ICommLink* comm_link, const AirSimSettings::EkfSetting* setting = nullptr)
            : board_(board), comm_link_(comm_link) // commlink is only temporary here
        {
            params_.initializeParameters(setting);
            freq_limiter_.initialize(334); // physics engine and the imu refresh at period 3ms ~ 333.33Hz
        }

        virtual void reset() override
        {
            IEkf::reset();

            freq_limiter_.reset();
            initializeFilter();

        }

        virtual void update() override
        {
            IEkf::update();

            // update the frequency limiter
            freq_limiter_.update();

            // if the wait is complete and it is time to update EKF, update EKF
            if (freq_limiter_.isWaitComplete())
                updateEKFInternal();
            
            // updateEKFInternal();
        }

        // only to debug and verify estimates 
        void setGroundTruthKinematics(const Kinematics::State* kinematics, const Environment* environment)
        {
            kinematics_ = kinematics;
            environment_ = environment;
        }

    private:
        // ---------------------------------------------------------------------
        // Internal functions
        // ---------------------------------------------------------------------
        
        // initialize filter
        void initializeFilter()
        {
            assignEkfMatrics();
            resetGlobalVariables();
        }

        void assignEkfMatrics()
        {
            Q_ = simple_flight::MatrixNWxNWf::Zero();
            // imu
            Q_(0, 0) = params_.accel.std_error.x()*params_.accel.std_error.x();
            Q_(1, 1) = params_.accel.std_error.y()*params_.accel.std_error.y();
            Q_(2, 2) = params_.accel.std_error.z()*params_.accel.std_error.z();
            Q_(3, 3) = params_.gyro.std_error.x()*params_.gyro.std_error.x();
            Q_(4, 4) = params_.gyro.std_error.y()*params_.gyro.std_error.y();
            Q_(5, 5) = params_.gyro.std_error.z()*params_.gyro.std_error.z();

            // biases
            Q_(6, 6)   = 0.0001f;
            Q_(7, 7)   = 0.0001f;
            Q_(8, 8)   = 0.0001f;
            Q_(9, 9)   = 0.0001f;
            Q_(10, 10) = 0.0001f;
            Q_(11, 11) = 0.0001f;
            Q_(12, 12) = 0.0001f;

            // gps
            R_gps_ = VectorMath::Matrix6x6f::Zero();
            R_gps_(0, 0) = params_.gps.std_error_position.x()*params_.gps.std_error_position.x();
            R_gps_(1, 1) = params_.gps.std_error_position.y()*params_.gps.std_error_position.y();
            R_gps_(2, 2) = params_.gps.std_error_position.z()*params_.gps.std_error_position.z();
            R_gps_(3, 3) = params_.gps.std_error_velocity.x()*params_.gps.std_error_velocity.x();
            R_gps_(4, 4) = params_.gps.std_error_velocity.y()*params_.gps.std_error_velocity.y();
            R_gps_(5, 5) = params_.gps.std_error_velocity.z()*params_.gps.std_error_velocity.z();

            // magnetometer
            R_mag_ = VectorMath::Matrix3x3f::Zero();
            R_mag_(0, 0) = params_.mag.std_error.x()*params_.mag.std_error.x();
            R_mag_(1, 1) = params_.mag.std_error.y()*params_.mag.std_error.y();
            R_mag_(2, 2) = params_.mag.std_error.z()*params_.mag.std_error.z();

            // barometer
            R_baro_ = params_.baro.std_error*params_.baro.std_error;

            // barometer
            R_pseudo_ = params_.pseudo_meas.quaternion_norm_R;

            // intialize the ekf states
            states_ = simple_flight::VectorNXf::Zero();
            states_(0)  = params_.initial_states.position.x();
            states_(1)  = params_.initial_states.position.y();
            states_(2)  = params_.initial_states.position.z();
            states_(3)  = params_.initial_states.linear_velocity.x();
            states_(4)  = params_.initial_states.linear_velocity.y();
            states_(5)  = params_.initial_states.linear_velocity.z();
            states_(6)  = params_.initial_states.quaternion.w();
            states_(7)  = params_.initial_states.quaternion.x();
            states_(8)  = params_.initial_states.quaternion.y();
            states_(9)  = params_.initial_states.quaternion.z();
            states_(10) = params_.initial_states.accel_bias.x();
            states_(11) = params_.initial_states.accel_bias.y();
            states_(12) = params_.initial_states.accel_bias.z();
            states_(13) = params_.initial_states.gyro_bias.x();
            states_(14) = params_.initial_states.gyro_bias.y();
            states_(15) = params_.initial_states.gyro_bias.z();
            states_(16) = params_.initial_states.baro_bias;
            
            // intitialize the ekf covariances
            error_covariance_ = simple_flight::MatrixNXxNXf::Zero();
            error_covariance_(0,0)   = pow(params_.initial_states_std_err.position.x(), 2);
            error_covariance_(1,1)   = pow(params_.initial_states_std_err.position.y(), 2);
            error_covariance_(2,2)   = pow(params_.initial_states_std_err.position.z(), 2);
            error_covariance_(3,3)   = pow(params_.initial_states_std_err.linear_velocity.x(), 2);
            error_covariance_(4,4)   = pow(params_.initial_states_std_err.linear_velocity.y(), 2);
            error_covariance_(5,5)   = pow(params_.initial_states_std_err.linear_velocity.z(), 2);
            error_covariance_(6,6)   = pow(params_.initial_states_std_err.quaternion.w(), 2);
            error_covariance_(7,7)   = pow(params_.initial_states_std_err.quaternion.x(), 2);
            error_covariance_(8,8)   = pow(params_.initial_states_std_err.quaternion.y(), 2);
            error_covariance_(9,9)   = pow(params_.initial_states_std_err.quaternion.z(), 2);
            error_covariance_(10,10) = pow(params_.initial_states_std_err.accel_bias.x(), 2);
            error_covariance_(11,11) = pow(params_.initial_states_std_err.accel_bias.y(), 2);
            error_covariance_(12,12) = pow(params_.initial_states_std_err.accel_bias.z(), 2);
            error_covariance_(13,13) = pow(params_.initial_states_std_err.gyro_bias.x(), 2);
            error_covariance_(14,14) = pow(params_.initial_states_std_err.gyro_bias.y(), 2);
            error_covariance_(15,15) = pow(params_.initial_states_std_err.gyro_bias.z(), 2);
            error_covariance_(16,16) = pow(params_.initial_states_std_err.baro_bias, 2);

        }

        void resetGlobalVariables()
        {
            // reset last update times
            last_times_.state_propagation = board_->micros();
            last_times_.cov_propagation = board_->micros();  

            // reset geo and magnetic global variables
            geodetic_converter_.setHome(environment_->getHomeGeoPoint());   
            VectorMath::Vector3f magnetic_field_true = EarthUtils::getMagField(environment_->getState().geo_point) * 1E4f; // in Gauss
            earth_mag_[0] = magnetic_field_true.x();
            earth_mag_[1] = magnetic_field_true.y();
            earth_mag_[2] = magnetic_field_true.z();

            // reset imu data buffer todo manage this in better way
            real_T accel[3];
            real_T gyro[3];
            bool is_new_and_valid = getImuData(accel, gyro);
            prev_imuData_.accel[0] = 0.0f;
            prev_imuData_.accel[1] = 0.0f;
            prev_imuData_.accel[2] = -9.80665f;
            prev_imuData_.gyro[0] = 0.0f;
            prev_imuData_.gyro[1] = 0.0f;
            prev_imuData_.gyro[2] = 0.0f;
            prev_imuData_.ang_acc[0] = 0.0f;
            prev_imuData_.ang_acc[1] = 0.0f;
            prev_imuData_.ang_acc[2] = 0.0f;
        }

        // this function updates at the frequency of EKF update
        void updateEKFInternal()
        {
            predictionStep();
            measurementUpdateStep();
        }

        // prediction step
        void predictionStep()
        {
            // the entire prediction step updates at the frequency of imu update
            // TODO later state propagation and covariance propagation can be decoupled!

            if(!board_->checkImuIfNew())
                return;

            real_T accel[3];
            real_T gyro[3];

            // check if the IMU gives new measurement and it is valid
            bool is_new_and_valid = getImuData(accel, gyro);

            if(!is_new_and_valid){
                return;
            }

            statePropagation(accel, gyro);
            covariancePropagation(accel, gyro);
        }

        // measurement update step
        void measurementUpdateStep()
        {
#if EKF_BARO_DIRECTIVE == 1
            barometerUpdate();
#else
#endif
#if EKF_MAGNETO_DIRECTIVE == 1
            magnetometerUpdate();
#else
#endif
#if EKF_GPS_DIRECTIVE == 1
            gpsUpdate();
#else
#endif
#if EKF_PSEUDOMEAS_DIRECTIVE == 1
            pseudoMeasurement();
#else
#endif
            eulerAnglesCovariancePropagation();
        }

        // state propagtion
        void statePropagation(real_T* accel, real_T* gyro)
        {
            // extract time step and update time
            TTimeDelta delta_T = (board_->micros() - last_times_.state_propagation) / 1.0E6; // in seconds
            float dt_real = static_cast<float>(delta_T);
            last_times_.state_propagation = board_->micros();

            // declare local variables
            float x_dot[simple_flight::NX];
            float x[simple_flight::NX];
            float u[simple_flight::NU];
            float uplus[simple_flight::NU];

            // extract the current ekf states
            for (int i=0; i<simple_flight::NX; i++){
                x[i] = states_(i);
            }

            // extract the current controls
            for (int i=0; i<3; i++){
                uplus[i] = accel[i];
                u[i] = prev_imuData_.accel[i];
                uplus[i+3] = gyro[i];
                u[i+3] = prev_imuData_.gyro[i];
            }

            // extract angular acceleration data
            float ang_accel[3];
            ang_accel[0] = prev_imuData_.ang_acc[0];
            ang_accel[1] = prev_imuData_.ang_acc[1];
            ang_accel[2] = prev_imuData_.ang_acc[2];

            // do prediction
            float x_predicted[simple_flight::NX];
            inertialNavigation(x_predicted, x, u, uplus, ang_accel, dt_real);

            // runge kutta integration
            // float x_predicted[17];
            // rungeKutta(x_predicted, x, u, dt_real);

            // heun integration
            // float x_predicted[simple_flight::NX];
            // heun(x_predicted, x, u, uplus, dt_real);

            // set the predicted states TODO: via an interface or after some checks
            for (int i=0; i<simple_flight::NX; i++){
                states_(i) = x_predicted[i];
            }

            // update imu data buffer
            prev_imuData_.accel[0] = accel[0];
            prev_imuData_.accel[1] = accel[1];
            prev_imuData_.accel[2] = accel[2];
            prev_imuData_.gyro[0] = gyro[0];
            prev_imuData_.gyro[1] = gyro[1];
            prev_imuData_.gyro[2] = gyro[2];
            prev_imuData_.ang_acc[0] = kinematics_->accelerations.angular.x();
            prev_imuData_.ang_acc[1] = kinematics_->accelerations.angular.y();
            prev_imuData_.ang_acc[2] = kinematics_->accelerations.angular.z();
        }

        // co-variance propagtion
        void covariancePropagation(real_T* accel, real_T* gyro)
        {
            // extract time step and update time
            TTimeDelta delta_T = (board_->micros() - last_times_.cov_propagation) / 1.0E6; // in seconds
            float dt_real = static_cast<float>(delta_T);
            last_times_.cov_propagation = board_->micros();

            // declare local variables
            float x[simple_flight::NX];
            float u[simple_flight::NU];
            simple_flight::MatrixNXxNXf A;
            simple_flight::MatrixNXxNXf A_finite;
            simple_flight::MatrixNXxNWf B_w;
            simple_flight::MatrixNXxNXf Phi;
            simple_flight::MatrixNXxNWf GammaB_w;
            simple_flight::MatrixNXxNXf P = error_covariance_;
            simple_flight::MatrixNXxNXf next_covariance;

            // extract the ekf states
            for (int i=0; i<simple_flight::NX; i++){
                x[i] = states_[i];
            }

            // extract the controls
            for (int i=0; i<3; i++){
                u[i] = accel[i];
                u[i+3] = gyro[i];
            }

            // evaluate A and B matrix
            evaluateA(&A, x, u);
            // evaluateFiniteDifferenceA(&A_finite, x, u);
            evaluateB_w(&B_w, x, u);

            evaluatePhiAndGamma_w(&Phi, &GammaB_w, &B_w, &A,  dt_real);
            // evaluatePhiAndGamma_w(&Phi, &GammaB_w, &B_w, &A_finite,  dt_real);

            // evaluate next covariance matrix
            next_covariance = Phi*P*Phi.transpose() + GammaB_w*Q_*GammaB_w.transpose();

            // set the new predicted covariance
            error_covariance_ = next_covariance;
        }

        void evaluatePhiAndGamma_w( simple_flight::MatrixNXxNXf* Phi, 
                                    simple_flight::MatrixNXxNWf* GammaB_w, 
                                    simple_flight::MatrixNXxNWf* B_w, 
                                    simple_flight::MatrixNXxNXf* A, 
                                    float dt_real)
        {
            // declare local variables
            simple_flight::MatrixNXxNXf identity = simple_flight::MatrixNXxNXf::Identity();
            simple_flight::MatrixNXxNXf A_square = (*A)*(*A);
            simple_flight::MatrixNXxNXf A_cube = A_square*(*A);
            simple_flight::MatrixNXxNXf A_forth = A_cube*(*A);
            simple_flight::MatrixNXxNXf A_fifth = A_forth*(*A);

            // calculate Phi matrix
            *Phi = identity
                   + (*A) * dt_real
                   + A_square * dt_real*dt_real/2
                   + A_cube * dt_real*dt_real*dt_real/6
                   + A_forth * dt_real*dt_real*dt_real*dt_real/24
                   + A_fifth * dt_real*dt_real*dt_real*dt_real*dt_real/120;

            // calculate GammaB_w matrix
            *GammaB_w = (identity * dt_real
                         + (*A) * dt_real*dt_real/2
                         + A_square * dt_real*dt_real*dt_real/6
                         + A_cube * dt_real*dt_real*dt_real*dt_real/24
                         + A_forth * dt_real*dt_real*dt_real*dt_real*dt_real/120
                         + A_fifth * dt_real*dt_real*dt_real*dt_real*dt_real*dt_real/720)*(*B_w);
        }

        // magnetometer update
        void magnetometerUpdate()
        {
            if(!board_->checkMagnetometerIfNew())
                return;
            
            real_T mag[3];

            // check if the magnetometer gives new measurement and it is valid
            bool is_valid = getMagnetometerData(mag);

            if(!is_valid){
                return;
            }

            // extract the ekf states
            float x[17];
            for (int i=0; i<17; i++){
                x[i] = states_(i);
            }

            // evaluate current measurement model
            float h_mag[3];
            evaluatehMag(h_mag, x, earth_mag_);

            // evaluate the C matrix
            simple_flight::Matrix3xNXf C_mag;
            evaluateCMag(&C_mag, x, earth_mag_);

            // calculate the Kalman gain matrix
            simple_flight::MatrixNXxNXf P = error_covariance_;
            VectorMath::Matrix3x3f inverse_term = (C_mag*P*C_mag.transpose() + R_mag_).inverse();
            simple_flight::MatrixNXx3f kalman_gain = P * C_mag.transpose() * inverse_term;

            // update states
            float x_corrected[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                float correction = kalman_gain(i, 0)*(mag[0] - h_mag[0])
                                  +kalman_gain(i, 1)*(mag[1] - h_mag[1])
                                  +kalman_gain(i, 2)*(mag[2] - h_mag[2]);
                x_corrected[i] = x[i] + correction;
            }

            // update covariance
            simple_flight::MatrixNXxNXf P_corrected;
            simple_flight::MatrixNXxNXf identity17x17 = simple_flight::MatrixNXxNXf::Identity();
            simple_flight::MatrixNXxNXf term = identity17x17 - kalman_gain*C_mag;
            P_corrected = term*P*term.transpose() + kalman_gain*R_mag_*kalman_gain.transpose();

            // write the new states and covariance matrix to global variables
            for (int i=0; i<simple_flight::NX; i++){
                states_(i) = x_corrected[i];
            }
            error_covariance_ = P_corrected;

        }

        // barometer update
        void barometerUpdate()
        {
            if(!board_->checkBarometerIfNew())
                return;

            real_T altitude[1];

            // check if the barometer gives new measurement and it is valid
            bool is_valid = getBarometerData(altitude);

            if(!is_valid)
            {
                return;
            }

            // extract the ekf states
            float x[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                x[i] = states_(i);
            }

            // evaluate the C matrix
            simple_flight::Matrix1xNXf C_baro;
            evaluateCBaro(&C_baro);

            // calculate the Kalman gain matrix
            simple_flight::MatrixNXxNXf P = error_covariance_;
            float inverse_term = 1.0f/(C_baro*P*C_baro.transpose() + R_baro_);
            simple_flight::MatrixNXx1f kalman_gain = P * C_baro.transpose() * inverse_term;

            // update states
            float x_corrected[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                x_corrected[i] = x[i] + kalman_gain[i]*(*altitude + x[2]);
            }

            // update covariances
            simple_flight::MatrixNXxNXf P_corrected;
            simple_flight::MatrixNXxNXf identity17x17 = simple_flight::MatrixNXxNXf::Identity();
            simple_flight::MatrixNXxNXf term = identity17x17 - kalman_gain*C_baro;
            P_corrected = term*P*term.transpose() + kalman_gain*R_baro_*kalman_gain.transpose();

            // write the new states and cavariance matrix to the global variables
            for (int i=0; i<simple_flight::NX; i++){
                states_(i) = x_corrected[i];
            }
            error_covariance_ = P_corrected;
        }

        // GPS update
        void gpsUpdate()
        {
            if(!board_->checkGpsIfNew())
                return;

            double pos[3];
            real_T vel[3];

            // check if the GPS gives new measurement and it is valid
            bool is_valid = getGpsData(pos, vel);

            if(!is_valid)
            {
                return;
            }

            // extract the ekf states
            float x[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                x[i] = states_(i);
            }

            // evaluate the C matrix
            simple_flight::Matrix6xNXf C_gps;
            evaluateCGps(&C_gps);

            // calculate the Kalman gain matrix
            simple_flight::MatrixNXxNXf P = error_covariance_;
            VectorMath::Matrix6x6f inverse_term = (C_gps*P*C_gps.transpose() + R_gps_).inverse();
            simple_flight::MatrixNXx6f kalman_gain = P * C_gps.transpose() * inverse_term;

            // update the states
            float x_corrected[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                float correction = kalman_gain(i, 0)*(pos[0] - x[0])
                                  +kalman_gain(i, 1)*(pos[1] - x[1])
                                  +kalman_gain(i, 2)*(pos[2] - x[2])
                                  +kalman_gain(i, 3)*(vel[0] - x[3])
                                  +kalman_gain(i, 4)*(vel[1] - x[4])
                                  +kalman_gain(i, 5)*(vel[2] - x[5]);
                x_corrected[i] = x[i] + correction;
            }

            // update the covariance matrix
            simple_flight::MatrixNXxNXf P_corrected;
            simple_flight::MatrixNXxNXf identity17x17 = simple_flight::MatrixNXxNXf::Identity();
            simple_flight::MatrixNXxNXf term = identity17x17 - kalman_gain*C_gps;
            P_corrected = term*P*term.transpose() + kalman_gain*R_gps_*kalman_gain.transpose();

            // write the new states and covariance matrix to the global variables
            for (int i=0; i<simple_flight::NX; i++){
                states_(i) = x_corrected[i];
            }
            error_covariance_ = P_corrected;

        }

        void pseudoMeasurement()
        {
            if(!board_->checkGpsIfNew())
                return;

            // extract the states
            float x[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                x[i] = states_(i);
            }

            // evaluate the current quaternion norm square
            float norm_square;
            norm_square =  x[6]*x[6] 
                         + x[7]*x[7]
                         + x[8]*x[8]
                         + x[9]*x[9]; 

            // evaluate the C matrix
            simple_flight::Matrix1xNXf C_pseudo;
            evaluateCPseudo(&C_pseudo, x);

            // calculate the Kalman gain matrix
            simple_flight::MatrixNXxNXf P = error_covariance_;
            float inverse_term = 1.0f/(C_pseudo*P*C_pseudo.transpose() + R_pseudo_);
            simple_flight::MatrixNXx1f kalman_gain = P* C_pseudo.transpose() * inverse_term;

            // update the ekf states
            float x_corrected[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                x_corrected[i] = x[i] + kalman_gain[i]*(1.0f - norm_square);
            }

            // covariance correction not done!!??? Is it correct??
            simple_flight::MatrixNXxNXf P_corrected;
            simple_flight::MatrixNXxNXf identity17x17 = simple_flight::MatrixNXxNXf::Identity();
            simple_flight::MatrixNXxNXf term = identity17x17 - kalman_gain*C_pseudo;
            P_corrected = term*P*term.transpose() + kalman_gain*R_pseudo*kalman_gain.transpose();

            // write the states to the global variable
            for (int i=0; i<simple_flight::NX; i++){
                states_(i) = x_corrected[i];
            }
            // error_covariance_ = P_corrected;
        }

        void eulerAnglesCovariancePropagation()
        {
            // extract the states
            float x[simple_flight::NX];
            for (int i=0; i<simple_flight::NX; i++){
                x[i] = states_(i);
            }

            // evaluate the C matrix
            VectorMath::Matrix3x4f C_euler;
            evaluateCEulerAngles(&C_euler, x);

            // define the matrices
            simple_flight::MatrixNXxNXf P = error_covariance_;
            VectorMath::Matrix3x3f P_euler_angles;
            VectorMath::Matrix4x4f P_quaternions;

            // map P onto P_quaternions
            P_quaternions(0, 0) = P(0+5, 0+5);
            P_quaternions(0, 1) = P(0+5, 1+5);
            P_quaternions(0, 2) = P(0+5, 2+5);
            P_quaternions(0, 3) = P(0+5, 3+5);

            P_quaternions(1, 0) = P(1+5, 0+5);
            P_quaternions(1, 1) = P(1+5, 1+5);
            P_quaternions(1, 2) = P(1+5, 2+5);
            P_quaternions(1, 3) = P(1+5, 3+5);

            P_quaternions(2, 0) = P(2+5, 0+5);
            P_quaternions(2, 1) = P(2+5, 1+5);
            P_quaternions(2, 2) = P(2+5, 2+5);
            P_quaternions(2, 3) = P(2+5, 3+5);

            P_quaternions(3, 0) = P(3+5, 0+5);
            P_quaternions(3, 1) = P(3+5, 1+5);
            P_quaternions(3, 2) = P(3+5, 2+5);
            P_quaternions(3, 3) = P(3+5, 3+5);

            P_euler_angles = C_euler*P_quaternions*C_euler.transpose();

            euler_angles_error_covariance_ = P_euler_angles;
        }

        // ---------------------------------------------------------------------
        // Measurement functions, reads measurement signal from board_
        // ---------------------------------------------------------------------

        // reads IMU data
        bool getImuData(real_T accel[3],
                        real_T gyro[3])
        {

#if EKF_GROUND_TRUTH_MEAS_DIRECTIVE == 1
            // >>>>> only for verification, with ground truth measurements
            VectorMath::Vector3f linear_acceleration = kinematics_->accelerations.linear - environment_->getState().gravity;
            // acceleration is in world frame so transform to body frame
            linear_acceleration = VectorMath::transformToBodyFrame(linear_acceleration,
                                                                   kinematics_->pose.orientation,
                                                                   true);
            accel[0] = linear_acceleration.x();
            accel[1] = linear_acceleration.y();
            accel[2] = linear_acceleration.z();
            gyro[0] = kinematics_->twist.angular.x();
            gyro[1] = kinematics_->twist.angular.y();
            gyro[2] = kinematics_->twist.angular.z();
            // >>>>>
#else
            board_->readImuData(accel, gyro);
#endif

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            // record the measurement signals
            measurement_(0) = accel[0];
            measurement_(1) = accel[1];
            measurement_(2) = accel[2];
            measurement_(3) = gyro[0];
            measurement_(4) = gyro[1];
            measurement_(5) = gyro[2];

            return true;

        }

        // reads GPS data
        bool getGpsData(double pos[3],
                        real_T vel[3])
        {

#if EKF_GROUND_TRUTH_MEAS_DIRECTIVE == 1
            // >>>>> only for verification, with ground truth measurements
            pos[0] = kinematics_->pose.position.x();
            pos[1] = kinematics_->pose.position.y();
            pos[2] = kinematics_->pose.position.z();
            // >>>>>
#else
            double geo[3];
            board_->readGpsData(geo, vel);

            // GeoPoint geopoint;
            // geopoint.latitude = geo[0];
            // geopoint.longitude = geo[1];
            // geopoint.altitude = geo[2];
            // GeoPoint geo_home = environment_->getHomeGeoPoint();

            // Vector3r measured =  EarthUtils::GeodeticToNedFast(geopoint, geo_home);

            GeoPoint geo_point;
            Vector3r ned_pos;
            geo_point.longitude = geo[0];
            geo_point.latitude  = geo[1];
            geo_point.altitude  = geo[2];
            geodetic_converter_.geodetic2Ned(geo_point, ned_pos);

            pos[0] = ned_pos[0];
            pos[1] = ned_pos[1];
            pos[2] = ned_pos[2];
            // pos[0] = geo[0];
            // pos[1] = geo[1];
            // pos[2] = geo[2];
#endif

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            // record the measurement signals
            measurement_(6) = pos[0];
            measurement_(7) = pos[1];
            measurement_(8) = pos[2];
            measurement_(9)  = vel[0];
            measurement_(10) = vel[1];
            measurement_(11) = vel[2];

            return true;

        }

        // reads barometer data
        bool getBarometerData(real_T* altitude)
        {
#if EKF_GROUND_TRUTH_MEAS_DIRECTIVE == 1
            altitude[0] = -1.0f*kinematics_->pose.position.z();
#else
            board_->readBarometerData(altitude);
#endif

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            measurement_(12) = altitude[0];

            return true;

        }
 
        // reads magnetometer data
        bool getMagnetometerData(real_T mag[3])
        {
            
#if EKF_GROUND_TRUTH_MEAS_DIRECTIVE == 1

#else
            board_->readMagnetometerData(mag);
#endif

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            measurement_(13) = mag[0];
            measurement_(14) = mag[1];
            measurement_(15) = mag[2];

            return true;

        }

    private:
        struct ImuDataBuffer
        {
            float accel[3];
            float gyro[3];
            float ang_acc[3];
        };
        struct LastTimes
        {
            TTimePoint state_propagation;
            TTimePoint cov_propagation;
        };
        
    private:
        // ---------------------------------------------------------------------
        // Class attritubes
        // ---------------------------------------------------------------------
        FrequencyLimiter freq_limiter_;
        const simple_flight::IBoard* board_;
        simple_flight::ICommLink* comm_link_;

        const Kinematics::State* kinematics_;
        const Environment* environment_;
        GeodeticConverter geodetic_converter_;
        float earth_mag_[3];
        
        LastTimes last_times_;
        ImuDataBuffer prev_imuData_;

        simple_flight::MatrixNWxNWf Q_;
        VectorMath::Matrix6x6f R_gps_;
        VectorMath::Matrix3x3f R_mag_;
        real_T R_baro_;
        real_T R_pseudo_;
    };

}
} //namespace
#endif
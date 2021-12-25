// Liscence info

#ifndef msr_airlib_AirSimSimpleEkf_hpp
#define msr_airlib_AirSimSimpleEkf_hpp

#include <exception>
#include <vector>
#include "firmware/interfaces/IBoard.hpp"
#include "firmware/interfaces/ICommLink.hpp"
#include "common/FrequencyLimiter.hpp"
#include "AirSimSimpleEkfBase.hpp"
#include "AirSimSimpleEkfModel.hpp"
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
        AirSimSimpleEkf(const simple_flight::IBoard* board, simple_flight::ICommLink* comm_link) //, const AirSimSettings::EKFSetting& setting = AirSimSettings::EKFSetting())
            : board_(board), comm_link_(comm_link) // commlink is only temporary here
        {
            // params_ = // get the EKF params from airsim settings. Implement this as a struct for now with constant members ! inspired by sensor model
            freq_limiter_.initialize(100); // physics engine and the imu refresh at period 3ms ~ 333.33Hz
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
            // if (freq_limiter_.isWaitComplete())
            //     updateEKFInternal();
            
            updateEKFInternal();
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
            simple_flight::SensorCharacteristics sensor_characteristics;
            simple_flight::EkfInitialStates initial_states;
            simple_flight::EkfInitialStdDevs initial_std_devs;

            // imu
            sensor_characteristics.accel_std_dev.x()   = 0.05f; // m/s^2
            sensor_characteristics.accel_std_dev.y()   = 0.05f; // m/s^2
            sensor_characteristics.accel_std_dev.z()   = 0.05f; // m/s^2
            sensor_characteristics.gyro_std_dev.x()    = 0.1f; // deg/s
            sensor_characteristics.gyro_std_dev.y()    = 0.1f; // deg/s
            sensor_characteristics.gyro_std_dev.z()    = 0.1f; // deg/s
            // gps
            sensor_characteristics.gps_pos_std_dev.x() = 5.0f; // m
            sensor_characteristics.gps_pos_std_dev.y() = 5.0f; // m
            sensor_characteristics.gps_pos_std_dev.z() = 10.0f; // m
            sensor_characteristics.gps_vel_std_dev.x() = 2.0f; // m/s
            sensor_characteristics.gps_vel_std_dev.y() = 2.0f; // m/s
            sensor_characteristics.gps_vel_std_dev.z() = 5.0f; // m/s
            // barometer
            sensor_characteristics.baro_std_dev        = 1.0f; // m
            // magnetomter
            sensor_characteristics.mag_std_dev.x()     = 0.01f; 
            sensor_characteristics.mag_std_dev.y()     = 0.01f; 
            sensor_characteristics.mag_std_dev.z()     = 0.01f; 

            // initial position
            initial_states.pos.x()                    = 2.0f;
            initial_states.pos.y()                    = 2.0f;
            initial_states.pos.z()                    = -10.0f + 2.0f;
            // initial velocity                   
            initial_states.vel.x()                    = 0.0f;
            initial_states.vel.y()                    = 0.0f;
            initial_states.vel.z()                    = 0.0f;
            // initial quaternions
            initial_states.quaternion.val4()          = 0.98929;
            initial_states.quaternion.x()             = 0.0789265;
            initial_states.quaternion.y()             = 0.0940609;
            initial_states.quaternion.z()             = 0.0789265;
            // initial biases      
            initial_states.accel_bias.x()             = 0.0f; 
            initial_states.accel_bias.y()             = 0.0f; 
            initial_states.accel_bias.z()             = 0.0f; 
            initial_states.gyro_bias.x()              = 0.0f; 
            initial_states.gyro_bias.y()              = 0.0f; 
            initial_states.gyro_bias.z()              = 0.0f; 
            initial_states.baro_bias                  = 0.0f; 

            // initial position standard deviations
            initial_std_devs.pos.x()                  = 5.0f;
            initial_std_devs.pos.y()                  = 5.0f;
            initial_std_devs.pos.z()                  = 10.0f;
            // initial velocity standard deviations                 
            initial_std_devs.vel.x()                  = 2.0f;
            initial_std_devs.vel.y()                  = 2.0f;
            initial_std_devs.vel.z()                  = 5.0f;
            // initial quaternions standard deviations    
            initial_std_devs.quaternion.val4()        = 0.1f;
            initial_std_devs.quaternion.x()           = 0.1f;
            initial_std_devs.quaternion.y()           = 0.1f;
            initial_std_devs.quaternion.z()           = 0.1f;
            // initial biases standard deviations    
            initial_std_devs.accel_bias.x()           = 0.05f;
            initial_std_devs.accel_bias.y()           = 0.05f;
            initial_std_devs.accel_bias.z()           = 0.05f;
            initial_std_devs.gyro_bias.x()            = 0.001f;
            initial_std_devs.gyro_bias.y()            = 0.001f;
            initial_std_devs.gyro_bias.z()            = 0.001f;
            initial_std_devs.baro_bias                = 0.1f;

            assignEkfMatrics(sensor_characteristics, initial_states, initial_std_devs);
            resetIntialGlobalVariables();
        }

        void assignEkfMatrics(const simple_flight::SensorCharacteristics& sensor,
                              const simple_flight::EkfInitialStates& initial_states,
                              const simple_flight::EkfInitialStdDevs& initial_std_devs)
        {
            // imu
            Q_(0, 0) = sensor.accel_std_dev.x()*sensor.accel_std_dev.x(); // accel_x
            Q_(1, 1) = sensor.accel_std_dev.y()*sensor.accel_std_dev.y(); // accel_y
            Q_(2, 2) = sensor.accel_std_dev.z()*sensor.accel_std_dev.z(); // accel_z
            Q_(3, 3) = sensor.gyro_std_dev.x()*sensor.gyro_std_dev.x() * M_PI*M_PI/32400; // gyro_x
            Q_(4, 4) = sensor.gyro_std_dev.y()*sensor.gyro_std_dev.y() * M_PI*M_PI/32400; // gyro_y
            Q_(5, 5) = sensor.gyro_std_dev.z()*sensor.gyro_std_dev.z() * M_PI*M_PI/32400; // gyro_z

            // biases
            Q_(6, 6)   = 0.0001f;
            Q_(7, 7)   = 0.0001f;
            Q_(8, 8)   = 0.0001f;
            Q_(9, 9)   = 0.0001f;
            Q_(10, 10) = 0.0001f;
            Q_(11, 11) = 0.0001f;
            Q_(12, 12) = 0.0001f;

            // gps
            R_gps_(0, 0) = sensor.gps_pos_std_dev.x()*sensor.gps_pos_std_dev.x(); // gps_pos_x
            R_gps_(1, 1) = sensor.gps_pos_std_dev.y()*sensor.gps_pos_std_dev.y(); // gps_pos_y
            R_gps_(2, 2) = sensor.gps_pos_std_dev.z()*sensor.gps_pos_std_dev.z(); // gps_pos_z
            R_gps_(3, 3) = sensor.gps_vel_std_dev.x()*sensor.gps_vel_std_dev.x(); // gps_vel_x
            R_gps_(4, 4) = sensor.gps_vel_std_dev.y()*sensor.gps_vel_std_dev.y(); // gps_vel_y
            R_gps_(5, 5) = sensor.gps_vel_std_dev.z()*sensor.gps_vel_std_dev.z(); // gps_vel_z

            // magnetometer
            R_mag_(0, 0) = sensor.mag_std_dev.x()*sensor.mag_std_dev.x(); // mag_x
            R_mag_(1, 1) = sensor.mag_std_dev.y()*sensor.mag_std_dev.y(); // mag_y
            R_mag_(2, 2) = sensor.mag_std_dev.z()*sensor.mag_std_dev.z(); // mag_z

            // barometer
            R_baro_ = sensor.baro_std_dev*sensor.baro_std_dev; // baro_alt

            // intialize the ekf states
            states_(0)  = initial_states.pos.x();
            states_(1)  = initial_states.pos.y();
            states_(2)  = initial_states.pos.z();
            states_(3)  = initial_states.vel.x();
            states_(4)  = initial_states.vel.y();
            states_(5)  = initial_states.vel.z();
            states_(6)  = initial_states.quaternion.val4();
            states_(7)  = initial_states.quaternion.x();
            states_(8)  = initial_states.quaternion.y();
            states_(9)  = initial_states.quaternion.z();
            states_(10) = initial_states.accel_bias.x();
            states_(11) = initial_states.accel_bias.y();
            states_(12) = initial_states.accel_bias.z();
            states_(13) = initial_states.gyro_bias.x();
            states_(14) = initial_states.gyro_bias.y();
            states_(15) = initial_states.gyro_bias.z();
            states_(16) = initial_states.baro_bias;

            // intitialize the ekf covariances
            covariance_(0,0)   = initial_std_devs.pos.x()*initial_std_devs.pos.x(); // x
            covariance_(1,1)   = initial_std_devs.pos.y()*initial_std_devs.pos.y(); // y
            covariance_(2,2)   = initial_std_devs.pos.z()*initial_std_devs.pos.z(); // z
            covariance_(3,3)   = initial_std_devs.vel.x()*initial_std_devs.vel.x(); // u
            covariance_(4,4)   = initial_std_devs.vel.y()*initial_std_devs.vel.y(); // v
            covariance_(5,5)   = initial_std_devs.vel.z()*initial_std_devs.vel.z(); // w     
            covariance_(6,6)   = initial_std_devs.quaternion.val4()*initial_std_devs.quaternion.val4(); // q0
            covariance_(7,7)   = initial_std_devs.quaternion.x()*initial_std_devs.quaternion.x(); // q1
            covariance_(8,8)   = initial_std_devs.quaternion.y()*initial_std_devs.quaternion.y(); // q2
            covariance_(9,9)   = initial_std_devs.quaternion.z()*initial_std_devs.quaternion.z(); // q3
            covariance_(10,10) = initial_std_devs.accel_bias.x()*initial_std_devs.accel_bias.x(); // 
            covariance_(11,11) = initial_std_devs.accel_bias.y()*initial_std_devs.accel_bias.y(); // 
            covariance_(12,12) = initial_std_devs.accel_bias.z()*initial_std_devs.accel_bias.z(); // 
            covariance_(13,13) = initial_std_devs.gyro_bias.x()*initial_std_devs.gyro_bias.x(); // 
            covariance_(14,14) = initial_std_devs.gyro_bias.y()*initial_std_devs.gyro_bias.y(); // 
            covariance_(15,15) = initial_std_devs.gyro_bias.z()*initial_std_devs.gyro_bias.z(); // 
            covariance_(16,16) = initial_std_devs.baro_bias*initial_std_devs.baro_bias; // 

        }

        void resetIntialGlobalVariables()
        {
            // reset last update times
            last_times_.state_propagation = board_->micros();
            last_times_.cov_propagation = board_->micros();  

            // reset geo and magnetic global variables
            geodetic_converter_.setHome(environment_->getHomeGeoPoint());   
            VectorMath::Vector3f magnetic_field_true = EarthUtils::getMagField(environment_->getState().geo_point) * 1E4f;
            earth_mag_[0] = magnetic_field_true.x();
            earth_mag_[1] = magnetic_field_true.y();
            earth_mag_[2] = magnetic_field_true.z();

            // reset imu data buffer
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
        }

        // state propagtion
        void statePropagation(real_T* accel, real_T* gyro)
        {
            // extract time step and update time
            TTimeDelta delta_T = (board_->micros() - last_times_.state_propagation) / 1.0E6; // in seconds
            float dt_real = static_cast<float>(delta_T);
            last_times_.state_propagation = board_->micros();

            // declare local variables
            float x_dot[17];
            float x[17];
            float u[6];
            float uplus[6];

            // extract the current ekf states
            for (int i=0; i<17; i++){
                x[i] = states_[i];
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
            float x_predicted[17];
            inertialNavigation(x_predicted, x, u, uplus, ang_accel, dt_real);

            // runge kutta integration
            // float x_predicted[17];
            // rungeKutta(x_predicted, x, u, dt_real);

            // heun integration
            // float x_predicted[17];
            // heun(x_predicted, x, u, uplus, dt_real);

            // set the predicted states TODO: via an interface or after some checks
            for (int i=0; i<17; i++){
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
            float x[17];
            float u[6];
            VectorMath::Matrix17x17f A;
            VectorMath::Matrix17x17f A_finite;
            VectorMath::Matrix17x13f B_w;
            VectorMath::Matrix17x17f Phi;
            VectorMath::Matrix17x13f GammaB_w;
            VectorMath::Matrix17x17f P = covariance_;
            VectorMath::Matrix17x17f next_covariance;

            // extract the ekf states
            for (int i=0; i<17; i++){
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
            covariance_ = next_covariance;
        }

        void evaluatePhiAndGamma_w( VectorMath::Matrix17x17f* Phi, 
                                    VectorMath::Matrix17x13f* GammaB_w, 
                                    VectorMath::Matrix17x13f* B_w, 
                                    VectorMath::Matrix17x17f* A, 
                                    float dt_real)
        {
            // declare local variables
            VectorMath::Matrix17x17f identity = VectorMath::Matrix17x17f::Identity();
            VectorMath::Matrix17x17f A_square = (*A)*(*A);
            VectorMath::Matrix17x17f A_cube = A_square*(*A);
            VectorMath::Matrix17x17f A_forth = A_cube*(*A);
            VectorMath::Matrix17x17f A_fifth = A_forth*(*A);

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
            VectorMath::Matrix3x17f C_mag;
            evaluateCMag(&C_mag, x, earth_mag_);

            // calculate the Kalman gain matrix
            VectorMath::Matrix17x17f P = covariance_;
            VectorMath::Matrix3x3f inverse_term = (C_mag*P*C_mag.transpose() + R_mag_).inverse();
            VectorMath::Matrix17x3f kalman_gain = P * C_mag.transpose() * inverse_term;

            // update states
            float x_corrected[17];
            for (int i=0; i<17; i++){
                float correction = kalman_gain(i, 0)*(mag[0] - h_mag[0])
                                  +kalman_gain(i, 1)*(mag[1] - h_mag[1])
                                  +kalman_gain(i, 2)*(mag[2] - h_mag[2]);
                x_corrected[i] = x[i] + correction;
            }

            // update covariance
            VectorMath::Matrix17x17f P_corrected;
            VectorMath::Matrix17x17f identity17x17 = VectorMath::Matrix17x17f::Identity();
            VectorMath::Matrix17x17f term = identity17x17 - kalman_gain*C_mag;
            P_corrected = term*P*term.transpose() + kalman_gain*R_mag_*kalman_gain.transpose();

            // write the new states and covariance matrix to global variables
            for (int i=0; i<17; i++){
                states_(i) = x_corrected[i];
            }
            covariance_ = P_corrected;

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
            float x[17];
            for (int i=0; i<17; i++){
                x[i] = states_(i);
            }

            // evaluate the C matrix
            VectorMath::Matrix1x17f C_baro;
            evaluateCBaro(&C_baro);

            // calculate the Kalman gain matrix
            VectorMath::Matrix17x17f P = covariance_;
            float inverse_term = 1.0f/(C_baro*P*C_baro.transpose() + R_baro_);
            VectorMath::Matrix17x1f kalman_gain = P * C_baro.transpose() * inverse_term;

            // update states
            float x_corrected[17];
            for (int i=0; i<17; i++){
                x_corrected[i] = x[i] + kalman_gain[i]*(*altitude + x[2]);
            }

            // update covariances
            VectorMath::Matrix17x17f P_corrected;
            VectorMath::Matrix17x17f identity17x17 = VectorMath::Matrix17x17f::Identity();
            VectorMath::Matrix17x17f term = identity17x17 - kalman_gain*C_baro;
            P_corrected = term*P*term.transpose() + kalman_gain*R_baro_*kalman_gain.transpose();

            // write the new states and cavariance matrix to the global variables
            for (int i=0; i<17; i++){
                states_(i) = x_corrected[i];
            }
            covariance_ = P_corrected;
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
            float x[17];
            for (int i=0; i<17; i++){
                x[i] = states_(i);
            }

            // evaluate the C matrix
            VectorMath::Matrix6x17f C_gps;
            evaluateCGps(&C_gps);

            // calculate the Kalman gain matrix
            VectorMath::Matrix17x17f P = covariance_;
            VectorMath::Matrix6x6f inverse_term = (C_gps*P*C_gps.transpose() + R_gps_).inverse();
            VectorMath::Matrix17x6f kalman_gain = P * C_gps.transpose() * inverse_term;

            // update the states
            float x_corrected[17];
            for (int i=0; i<17; i++){
                float correction = kalman_gain(i, 0)*(pos[0] - x[0])
                                  +kalman_gain(i, 1)*(pos[1] - x[1])
                                  +kalman_gain(i, 2)*(pos[2] - x[2])
                                  +kalman_gain(i, 3)*(vel[0] - x[3])
                                  +kalman_gain(i, 4)*(vel[1] - x[4])
                                  +kalman_gain(i, 5)*(vel[2] - x[5]);
                x_corrected[i] = x[i] + correction;
            }

            // update the covariance matrix
            VectorMath::Matrix17x17f P_corrected;
            VectorMath::Matrix17x17f identity17x17 = VectorMath::Matrix17x17f::Identity();
            VectorMath::Matrix17x17f term = identity17x17 - kalman_gain*C_gps;
            P_corrected = term*P*term.transpose() + kalman_gain*R_gps_*kalman_gain.transpose();

            // write the new states and covariance matrix to the global variables
            for (int i=0; i<17; i++){
                states_(i) = x_corrected[i];
            }
            covariance_ = P_corrected;

        }

        void pseudoMeasurement()
        {
            if(!board_->checkGpsIfNew())
                return;

            // extract the states
            float x[17];
            for (int i=0; i<17; i++){
                x[i] = states_[i];
            }

            // evaluate the current quaternion norm square
            float norm_square;
            norm_square =  x[6]*x[6] 
                         + x[7]*x[7]
                         + x[8]*x[8]
                         + x[9]*x[9]; 

            // evaluate the C matrix
            VectorMath::Matrix1x17f C_pseudo = VectorMath::Matrix1x17f::Zero();
            evaluateCPseudo(&C_pseudo, x);

            // calculate the Kalman gain matrix
            float R_pseudo = 0.0001f;
            VectorMath::Matrix17x17f P = covariance_;
            float inverse_term = 1.0f/(C_pseudo*P*C_pseudo.transpose() + R_pseudo);
            VectorMath::Matrix17x1f kalman_gain = P* C_pseudo.transpose() * inverse_term;

            // update the ekf states
            float x_corrected[17];
            for (int i=0; i<17; i++){
                x_corrected[i] = x[i] + kalman_gain[i]*(1.0f - norm_square);
            }

            // covariance correction not done!!??? Is it correct??
            // VectorMath::Matrix17x17f P_corrected;
            // VectorMath::Matrix17x17f identity17x17 = VectorMath::Matrix17x17f::Identity();
            // VectorMath::Matrix17x17f term = identity17x17 - kalman_gain*C_pseudo;
            // P_corrected = term*P*term.transpose() + kalman_gain*R_pseudo*kalman_gain.transpose();

            // write the states to the global variable
            for (int i=0; i<17; i++){
                states_(i) = x_corrected[i];
            }
            // covariance_ = P_corrected;
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

        struct ImuDataBuffer
        {
            float accel[3];
            float gyro[3];
            float ang_acc[3];
        };
        
    private:
        // ---------------------------------------------------------------------
        // Class attritubes
        // ---------------------------------------------------------------------
        // parameter struct instance
        // params_;
        // frequency limiter for EKF
        FrequencyLimiter freq_limiter_;
        // the flight board instance
        const simple_flight::IBoard* board_;
        simple_flight::ICommLink* comm_link_; // commlink is only temporary here, later transfer logging to OffBoardApi via StateEstimator 

        const Kinematics::State* kinematics_;
        const Environment* environment_;
        GeodeticConverter geodetic_converter_;

        VectorMath::Matrix13x13f Q_;
        VectorMath::Matrix6x6f R_gps_;
        VectorMath::Matrix3x3f R_mag_;
        real_T R_baro_;

        ImuDataBuffer prev_imuData_;
        float earth_mag_[3];

    };

}
} //namespace
#endif
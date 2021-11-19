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
            freq_limiter_.initialize(334); // physics engine and the imu refresh at period 3ms ~ 333.33Hz
        }

        virtual void reset() override
        {
            IEkf::reset();

            freq_limiter_.reset();
            initialize();

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

    private:
        // ---------------------------------------------------------------------
        // Internal functions
        // ---------------------------------------------------------------------
            // initialize filter
        void initialize()
        {
            // intiaalize with zero position and unity quaternion ! temporary | TODO do using setters 
            states_ = VectorMath::Vector17f::Zero();
            states_[6] = 1.0f;
            covariance_ = VectorMath::Matrix17x17f::Identity()*0.01f;
            //covariance_(2,2) = 1.0f;
            Q_ = VectorMath::Matrix13x13f::Identity()*0.01f;
            Q_(0, 0) = 0.25f;
            Q_(1, 1) = 0.25f;
            Q_(2, 2) = 0.25f;
            Q_(3, 3) = 0.25f;
            Q_(4, 4) = 0.25f;
            Q_(5, 5) = 0.25f;

            last_times_.state_propagation = board_->micros();
            last_times_.cov_propagation = board_->micros();
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

            // imu updates at higher frequency than the EKF. 
            // !Downsampling, needed or automatic? does it have any side effects?

            // check if the IMU gives new measurement and it is valid
            bool is_new_and_valid = getImuData(accel, gyro);

            if(!is_new_and_valid){
                return;
            }

            statePropagation(accel, gyro);
            covariancePropagation(accel, gyro);

            /*std::ostringstream imu_str;
            imu_str << accel[2];
            std::string messgae = "   prediction step / imu step called (z-accelerometer) " + imu_str.str();
            comm_link_->log(messgae);*/

            // std::ostringstream prediction_str;
            // prediction_str << "Prediction: "   << states_[0] 
            //                             << '\t' << states_[1] 
            //                             << '\t' << states_[2]
            //                             << '\t' << states_[3] 
            //                             << '\t' << states_[4]
            //                             << '\t' << states_[5]
            //                             << '\t' << states_[6]
            //                             << '\t' << states_[7]
            //                             << '\t' << states_[8]
            //                             << '\t' << states_[9]
            //                             << '\t' ;
            // std::string messgae = prediction_str.str();
            // comm_link_->log(messgae);
        }

        // measurement update step
        void measurementUpdateStep()
        {
            magnetometerUpdate();
            barometerUpdate();
            gpsUpdate();
        }

        // state propagtion
        void statePropagation(real_T* accel, real_T* gyro)
        {
            float x_dot[10];
            float x[17];
            float u[6];

            // extract the states
            for (int i=0; i<17; i++){
                x[i] = states_[i];
            }

            // extract the controls
            for (int i=0; i<3; i++){
                u[i] = accel[i];
                u[i+3] = gyro[i];
            }

            evaluateStateDot(x_dot,x,u);

            TTimeDelta delta_T = (board_->micros() - last_times_.state_propagation) / 1.0E6; // in seconds
            last_times_.state_propagation = board_->micros();

            // euler forward integration
            float x_predicted[17];
            for (int i=0; i<10; i++){
                x_predicted[i] = x[i] + static_cast<float>(delta_T*x_dot[i]);
            }
            for (int i=10; i<17; i++){
                x_predicted[i] = x[i];
            }

            // set the predicted states TODO: via an interface or after some checks
            for (int i=0; i<17; i++){
                states_[i] = x_predicted[i];
            }
        }

        // co-variance propagtion
        void covariancePropagation(real_T* accel, real_T* gyro)
        {
            VectorMath::Matrix17x17f A;
            VectorMath::Matrix17x13f B_w;
            float x[17];
            float u[6];

            // extract the states
            for (int i=0; i<17; i++){
                x[i] = states_[i];
            }

            // extract the controls
            for (int i=0; i<3; i++){
                u[i] = accel[i];
                u[i+3] = gyro[i];
            }

            evaluateA(&A, x, u);
            evaluateB_w(&B_w, x, u);

            TTimeDelta delta_T = (board_->micros() - last_times_.cov_propagation) / 1.0E6; // in seconds
            last_times_.cov_propagation = board_->micros();

            VectorMath::Matrix17x17f Phi;
            VectorMath::Matrix17x13f GammaB_w;
            evaluatePhiAndGamma_w(&Phi, &GammaB_w, &B_w, &A,  delta_T);

            VectorMath::Matrix17x17f next_covariance;
            next_covariance = Phi*covariance_*Phi.transpose() + GammaB_w*Q_*GammaB_w.transpose();

            // set the new predicted covariance
            covariance_ = next_covariance;

        }

        // magnetometer update
        void magnetometerUpdate()
        {
            if(!board_->checkMagnetometerIfNew())
                return;
            
            // the updates at the frequency of magnetometer signal update

            real_T mag[3];

            // check if the magnetometer gives new measurement and it is valid
            bool is_valid = getMagnetometerData(mag);

            if(!is_valid){
                return;
            }

            // auto C = dh_mag_dx();

            // auto K = ...

            // std::ostringstream mag_str;
            // mag_str << mag[0];
            // std::string messgae = "        magnetometer step called (x-magnetic flux) " + mag_str.str();
            // comm_link_->log(messgae);
        }

        // barometer update
        void barometerUpdate()
        {
            if(!board_->checkBarometerIfNew())
                return;

            // the updates at the frequency of barometer signal update

            real_T altitude[1];

            // check if the barometer gives new measurement and it is valid
            bool is_valid = getBarometerData(altitude);

            if(!is_valid)
            {
                return;
            }

            // std::ostringstream alt_str;
            // alt_str << altitude[0];
            // std::string messgae = "        barometer step called (baro altitude) " + alt_str.str();
            // comm_link_->log(messgae);
        }

        // GPS update
        void gpsUpdate()
        {
            if(!board_->checkGpsIfNew())
                return;

            // the updates at the frequency of GPS signal update

            double geo[3];
            real_T vel[3];

            // check if the GPS gives new measurement and it is valid
            bool is_valid = getGpsData(geo, vel);

            if(!is_valid)
            {
                return;
            }

            // std::ostringstream gps_str;
            // gps_str << geo[2];
            // std::string messgae = "        gps step called (gps altitude) " + gps_str.str();
            // comm_link_->log(messgae);
        }

        void evaluatePhiAndGamma_w( VectorMath::Matrix17x17f* Phi, 
                                    VectorMath::Matrix17x13f* GammaB_w, 
                                    VectorMath::Matrix17x13f* B_w, 
                                    VectorMath::Matrix17x17f* A, 
                                    TTimeDelta delta_T)
        {
            // calculate Phi based on the jacobian for some iteration
            VectorMath::Matrix17x17f identity = VectorMath::Matrix17x17f::Identity();
            VectorMath::Matrix17x17f A_square = *A*(*A);

            *Phi = identity
                   + *A * delta_T
                   + A_square * delta_T*delta_T/2;

            *GammaB_w = (identity
                         + identity * delta_T
                         + *A * delta_T*delta_T/2)*(*B_w);
        }

        // ---------------------------------------------------------------------
        // Measurement functions, reads measurement signal from board_
        // ---------------------------------------------------------------------

        // reads IMU data
        bool getImuData(real_T accel[3],
                        real_T gyro[3])
        {
            board_->readImuData(accel, gyro);

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            return true;

        }

        // reads magnetometer data
        bool getMagnetometerData(real_T mag[3])
        {
            board_->readMagnetometerData(mag);

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            return true;

        }

        // reads barometer data
        bool getBarometerData(real_T* altitude)
        {
            board_->readBarometerData(altitude);

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            return true;

        }

        // reads GPS data
        bool getGpsData(double geo[3],
                        real_T vel[3])
        {
            board_->readGpsData(geo, vel);

            // check if the signal has all data that is valid, else return false
            // TODO: check if at least a subset of data is valid

            return true;

        }

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

    };

}
} //namespace
#endif
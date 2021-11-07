// Liscence info

#ifndef msr_airlib_AirSimSimpleEkf_hpp
#define msr_airlib_AirSimSimpleEkf_hpp

#include <exception>
#include <vector>
#include "interfaces/IBoard.hpp"
#include "common/FrequencyLimiter.hpp"
#include "AirSimSimpleEkfBase.hpp"

// #include "firmware/Params.hpp"
// #include "common/Common.hpp"
// #include "common/ClockFactory.hpp"
// #include "physics/Kinematics.hpp"                                                                   

namespace msr
{
namespace airlib
{

    class AirSimSimpleEkf : public AirSimSimpleEkfBase
        , private AirSimSimpleEkfModel
    {
    public:
        // Constructor
        AirSimSimpleEkf(const simple_flight::IBoard* board)//, const AirSimSettings::EKFSetting& setting = AirSimSettings::EKFSetting())
            : board_(board)
        {
            // params_ = // get the EKF params from airsim settings. Implement this as a struct for now with constant members ! inspired by sensor model
            freq_limiter_.initialize(100); // physics engine and the imu refresh at period 3m ~ 333.33Hz. EKF tops at frequency 100Hz
        }

        virtual void update() override
        {
            IUpdatable::update();

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

            statePropagation();
            covariancePropagation();
        }

        // measurement update step
        void measurementUpdateStep()
        {
            magnetometerUpdate();
            barometerUpdate();
            gpsUpdate();
        }

        // state propagtion
        void statePropagation()
        {
            // auto Phi = calculatePhi();
            // auto Gamma_w = calculateGamma_w();
        }

        // co-variance propagtion
        void covariancePropagation()
        {
            // auto Phi = calculatePhi();
            // auto Gamma_w = calculateGamma_w();
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
        }

        // barometer update
        void barometerUpdate()
        {
            if(!board_->checkBarometerIfNew())
                return;

            // the updates at the frequency of barometer signal update

            real_T* altitude;

            // check if the barometer gives new measurement and it is valid
            bool is_valid = getBarometerData(altitude);

            if(!is_valid)
            {
                return;
            }
        }

        // GPS update
        void gpsUpdate()
        {
            if(!board_->checkGpsIfNew())
                return;

            // the updates at the frequency of GPS signal update

            real_T geo[3];
            real_T vel[3];

            // check if the GPS gives new measurement and it is valid
            bool is_valid = getGpsData(geo, vel);

            if(!is_valid)
            {
                return;
            }
        }

        void calculatePhi()
        {
            // calculate Phi based on the jacobian for some iteration
        }

        void calculateGamma_w()
        {
            // calculate Gamma_w based on the jacobian for some iteration
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
        bool getGpsData(real_T geo[3],
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

    };

}
} //namespace
#endif
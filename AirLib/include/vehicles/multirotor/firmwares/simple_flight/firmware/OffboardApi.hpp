#pragma once

#include <cstdint>
#include "interfaces/ICommLink.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IOffboardApi.hpp"
#include "interfaces/IUpdatable.hpp"
#include "interfaces/CommonStructs.hpp"
#include "RemoteControl.hpp"
#include "Params.hpp"

namespace simple_flight
{

class OffboardApi : public IUpdatable
    , public IOffboardApi
{
public:
    OffboardApi(const Params* params, const IBoardClock* clock, const IBoardInputPins* board_inputs,
                IStateEstimator* state_estimator, ICommLink* comm_link)
        : params_(params), rc_(params, clock, board_inputs, &vehicle_state_, state_estimator, comm_link), state_estimator_(state_estimator), comm_link_(comm_link), clock_(clock), landed_(true)
    {
    }

    virtual void reset() override
    {
        IUpdatable::reset();

        vehicle_state_.setState(params_->default_vehicle_state, state_estimator_->getGeoPoint());
        rc_.reset();
        has_api_control_ = false;
        landed_ = true;
        takenoff_ = false;
        goal_timestamp_ = clock_->millis();
        updateGoalFromRc();
    }

    virtual void update() override
    {
        IUpdatable::update();

        rc_.update();
        if (!has_api_control_)
            updateGoalFromRc();
        else {
            if (takenoff_ &&
                (clock_->millis() - goal_timestamp_ > params_->api_goal_timeout)) {
                if (!is_api_timedout_) {
                    comm_link_->log("API call was not received, entering hover mode for safety");
                    goal_mode_ = GoalMode::getPositionMode();
                    goal_ = Axis4r::xyzToAxis4(state_estimator_->getPosition(), true);
                    is_api_timedout_ = true;
                }

                //do not update goal_timestamp_
            }
        }
        //else leave the goal set by IOffboardApi API

        detectLanding();
        detectTakingOff();

        logEkfValues();
    }

    /**************** IOffboardApi ********************/

    virtual const Axis4r& getGoalValue() const override
    {
        return goal_;
    }

    virtual const GoalMode& getGoalMode() const override
    {
        return goal_mode_;
    }

    virtual bool canRequestApiControl(std::string& message) override
    {
        if (rc_.allowApiControl())
            return true;
        else {
            message = "Remote Control switch position disallows API control";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }
    virtual bool hasApiControl() override
    {
        return has_api_control_;
    }
    virtual bool requestApiControl(std::string& message) override
    {
        if (canRequestApiControl(message)) {
            has_api_control_ = true;

            //initial value from RC for smooth transition
            updateGoalFromRc();

            comm_link_->log("requestApiControl was successful", ICommLink::kLogLevelInfo);

            return true;
        }
        else {
            comm_link_->log("requestApiControl failed", ICommLink::kLogLevelError);
            return false;
        }
    }
    virtual void releaseApiControl() override
    {
        has_api_control_ = false;
        comm_link_->log("releaseApiControl was successful", ICommLink::kLogLevelInfo);
    }
    virtual bool setGoalAndMode(const Axis4r* goal, const GoalMode* goal_mode, std::string& message) override
    {
        if (has_api_control_) {
            if (goal != nullptr)
                goal_ = *goal;
            if (goal_mode != nullptr)
                goal_mode_ = *goal_mode;
            goal_timestamp_ = clock_->millis();
            is_api_timedout_ = false;
            return true;
        }
        else {
            message = "requestApiControl() must be called before using API control";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }

    virtual bool arm(std::string& message) override
    {
        if (has_api_control_) {
            if (vehicle_state_.getState() == VehicleStateType::Armed) {
                message = "Vehicle is already armed";
                comm_link_->log(message, ICommLink::kLogLevelInfo);
                return true;
            }
            else if ((vehicle_state_.getState() == VehicleStateType::Inactive || vehicle_state_.getState() == VehicleStateType::Disarmed || vehicle_state_.getState() == VehicleStateType::BeingDisarmed)) {

                vehicle_state_.setState(VehicleStateType::Armed, state_estimator_->getHomeGeoPoint());
                goal_ = Axis4r(0, 0, 0, params_->rc.min_angling_throttle);
                goal_mode_ = GoalMode::getAllRateMode();

                message = "Vehicle is armed";
                comm_link_->log(message, ICommLink::kLogLevelInfo);
                return true;
            }
            else {
                message = "Vehicle cannot be armed because it is not in Inactive, Disarmed or BeingDisarmed state";
                comm_link_->log(message, ICommLink::kLogLevelError);
                return false;
            }
        }
        else {
            message = "Vehicle cannot be armed via API because API has not been given control";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }

    virtual bool disarm(std::string& message) override
    {
        if (has_api_control_ && (vehicle_state_.getState() == VehicleStateType::Active || vehicle_state_.getState() == VehicleStateType::Armed || vehicle_state_.getState() == VehicleStateType::BeingArmed)) {

            vehicle_state_.setState(VehicleStateType::Disarmed);
            goal_ = Axis4r(0, 0, 0, 0);
            goal_mode_ = GoalMode::getAllRateMode();

            message = "Vehicle is disarmed";
            comm_link_->log(message, ICommLink::kLogLevelInfo);
            return true;
        }
        else {
            message = "Vehicle cannot be disarmed because it is not in Active, Armed or BeingArmed state";
            comm_link_->log(message, ICommLink::kLogLevelError);
            return false;
        }
    }

    virtual VehicleStateType getVehicleState() const override
    {
        return vehicle_state_.getState();
    }

    virtual const IStateEstimator& getStateEstimator() override
    {
        return *state_estimator_;
    }

    virtual GeoPoint getHomeGeoPoint() const override
    {
        return state_estimator_->getHomeGeoPoint();
    }

    virtual GeoPoint getGeoPoint() const override
    {
        return state_estimator_->getGeoPoint();
    }

    virtual bool getLandedState() const override
    {
        return landed_;
    }

private:

    void logEkfValues()
    {
        // additional logging of vehicle states, TODO implement a separate log!
        std::ostringstream log_msg;
        log_msg << clock_->millis() << '\t'
        // ground truth mesurement signals
                << state_estimator_->getTrueMeasurements().accel.x() << '\t'
                << state_estimator_->getTrueMeasurements().accel.y() << '\t'
                << state_estimator_->getTrueMeasurements().accel.z() << '\t'
                << state_estimator_->getTrueMeasurements().gyro.x() << '\t'
                << state_estimator_->getTrueMeasurements().gyro.y() << '\t'
                << state_estimator_->getTrueMeasurements().gyro.z() << '\t'
        // noisy mesurement signals
                << state_estimator_->getEkfMeasurements().accel.x() << '\t'
                << state_estimator_->getEkfMeasurements().accel.y() << '\t'
                << state_estimator_->getEkfMeasurements().accel.z() << '\t'
                << state_estimator_->getEkfMeasurements().gyro.x() << '\t'
                << state_estimator_->getEkfMeasurements().gyro.y() << '\t'
                << state_estimator_->getEkfMeasurements().gyro.z() << '\t'
                << state_estimator_->getEkfMeasurements().gps_position.x() << '\t'
                << state_estimator_->getEkfMeasurements().gps_position.y() << '\t'
                << state_estimator_->getEkfMeasurements().gps_position.z() << '\t'
                << state_estimator_->getEkfMeasurements().gps_velocity.x() << '\t'
                << state_estimator_->getEkfMeasurements().gps_velocity.y() << '\t'
                << state_estimator_->getEkfMeasurements().gps_velocity.z() << '\t'
                << state_estimator_->getEkfMeasurements().baro_altitude << '\t'
                << state_estimator_->getEkfMeasurements().magnetic_flux.x() << '\t'
                << state_estimator_->getEkfMeasurements().magnetic_flux.y() << '\t'
                << state_estimator_->getEkfMeasurements().magnetic_flux.z() << '\t'
        // ground truth states
                << state_estimator_->getKinematicsEstimated().position.x() << '\t'
                << state_estimator_->getKinematicsEstimated().position.y() << '\t'
                << state_estimator_->getKinematicsEstimated().position.z() << '\t'
                << state_estimator_->getKinematicsEstimated().orientation.val4() << '\t' // ATTENSION val4 is w when converted to axis4r of simple_flight
                << state_estimator_->getKinematicsEstimated().orientation.x() << '\t'
                << state_estimator_->getKinematicsEstimated().orientation.y() << '\t'
                << state_estimator_->getKinematicsEstimated().orientation.z() << '\t'
                << state_estimator_->getAngles().pitch() << '\t'
                << state_estimator_->getAngles().roll() << '\t'
                << state_estimator_->getAngles().yaw() << '\t'
                << state_estimator_->getKinematicsEstimated().linear_velocity.x() << '\t'
                << state_estimator_->getKinematicsEstimated().linear_velocity.y() << '\t'
                << state_estimator_->getKinematicsEstimated().linear_velocity.z() << '\t'
        // estimated states
                << state_estimator_->getEkfKinematicsEstimated().position.x() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().position.y() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().position.z() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().orientation.val4() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().orientation.x() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().orientation.y() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().orientation.z() << '\t'
                << state_estimator_->getEkfAngles().pitch() << '\t'
                << state_estimator_->getEkfAngles().roll() << '\t'
                << state_estimator_->getEkfAngles().yaw() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().linear_velocity.x() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().linear_velocity.y() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().linear_velocity.z() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.accel.x() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.accel.y() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.accel.z() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.gyro.x() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.gyro.y() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.gyro.z() << '\t'
                << state_estimator_->getEkfKinematicsEstimated().sensor_bias.barometer << '\t'
        // covariances
                << state_estimator_->getEkfPositionCovariance().x() << '\t'
                << state_estimator_->getEkfPositionCovariance().y() << '\t'
                << state_estimator_->getEkfPositionCovariance().z() << '\t'
                << state_estimator_->getEkfLinearVelocityCovariance().x() << '\t'
                << state_estimator_->getEkfLinearVelocityCovariance().y() << '\t'
                << state_estimator_->getEkfLinearVelocityCovariance().z() << '\t'
                << state_estimator_->getEkfOrientationCovariance().x() << '\t'
                << state_estimator_->getEkfOrientationCovariance().y() << '\t'
                << state_estimator_->getEkfOrientationCovariance().z() << '\t'
                << state_estimator_->getEkfOrientationCovariance().val4() << '\t'
                << state_estimator_->getEkfImuBiasCovariance().x() << '\t'
                << state_estimator_->getEkfImuBiasCovariance().y() << '\t'
                << state_estimator_->getEkfImuBiasCovariance().z() << '\t'
                << state_estimator_->getEkfGyroBiasCovariance().x() << '\t'
                << state_estimator_->getEkfGyroBiasCovariance().y() << '\t'
                << state_estimator_->getEkfGyroBiasCovariance().z() << '\t'
                << state_estimator_->getEkfBaroBiasCovariance() << '\t'
        // quaternion norm
                << state_estimator_->getEkfOrientationNorm() << '\t'
        // off-diag quaternion covariance
                << state_estimator_->getEkfOrientationOffDiagCovariance().at(0) << '\t'
                << state_estimator_->getEkfOrientationOffDiagCovariance().at(1) << '\t'
                << state_estimator_->getEkfOrientationOffDiagCovariance().at(2) << '\t'
                << state_estimator_->getEkfOrientationOffDiagCovariance().at(3) << '\t'
                << state_estimator_->getEkfOrientationOffDiagCovariance().at(4) << '\t'
                << state_estimator_->getEkfOrientationOffDiagCovariance().at(5) << '\t'
        // gyro bias quaternion covariance
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(0) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(1) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(2) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(3) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(4) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(5) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(6) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(7) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(8) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(9) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(10) << '\t'
                << state_estimator_->getEkfOrientationGyroBiasCovariance().at(11) << '\t';

        std::string message = log_msg.str();
        comm_link_->log(message);
    }

    void updateGoalFromRc()
    {
        goal_ = rc_.getGoalValue();
        goal_mode_ = rc_.getGoalMode();
    }

    void detectLanding()
    {

        // if we are not trying to move by setting motor outputs
        if (takenoff_) {
            //if (!isGreaterThanArmedThrottle(goal_.throttle())) {
            float checkThrottle = rc_.getMotorOutput();
            if (!isGreaterThanArmedThrottle(checkThrottle)) {
                // and we are not currently moving (based on current velocities)
                auto angular = state_estimator_->getAngularVelocity();
                auto velocity = state_estimator_->getLinearVelocity();
                if (isAlmostZero(angular.roll()) && isAlmostZero(angular.pitch()) && isAlmostZero(angular.yaw()) &&
                    isAlmostZero(velocity.x()) && isAlmostZero(velocity.y()) && isAlmostZero(velocity.z())) {
                    // then we must be landed...
                    landed_ = true;
                    takenoff_ = false;
                }
            }
        }
    }

    void detectTakingOff()
    {
        // if we are not trying to move by setting motor outputs
        if (!takenoff_) {
            float checkThrottle = rc_.getMotorOutput();
            //TODO: better handling of landed & takenoff states
            if (isGreaterThanArmedThrottle(checkThrottle) &&
                std::abs(state_estimator_->getLinearVelocity().z()) > 0.01f) {
                takenoff_ = true;
                landed_ = false;
            }
        }
    }

    bool isAlmostZero(float v)
    {
        return std::abs(v) < kMovementTolerance;
    }
    bool isGreaterThanArmedThrottle(float throttle)
    {
        return throttle > params_->min_armed_throttle();
    }

private:
    const TReal kMovementTolerance = (TReal)0.08;
    const Params* params_;
    RemoteControl rc_;
    IStateEstimator* state_estimator_;
    ICommLink* comm_link_;
    const IBoardClock* clock_;

    VehicleState vehicle_state_;

    Axis4r goal_;
    GoalMode goal_mode_;
    uint64_t goal_timestamp_;

    bool has_api_control_;
    bool is_api_timedout_;
    bool landed_, takenoff_;
};

} //namespace
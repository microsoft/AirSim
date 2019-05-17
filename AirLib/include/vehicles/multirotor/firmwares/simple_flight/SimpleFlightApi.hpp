// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleFlightDroneController_hpp
#define msr_airlib_SimpleFlightDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "firmware/Firmware.hpp"
#include "AirSimSimpleFlightBoard.hpp"
#include "AirSimSimpleFlightCommLink.hpp"
#include "AirSimSimpleFlightEstimator.hpp"
#include "AirSimSimpleFlightCommon.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

//TODO: we need to protect contention between physics thread and API server thread

namespace msr { namespace airlib {

class SimpleFlightApi : public MultirotorApiBase {

public:
    SimpleFlightApi(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSetting* vehicle_setting)
        : vehicle_params_(vehicle_params)
    {
        readSettings(*vehicle_setting);

        //TODO: set below properly for better high speed safety
        safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;

        //create sim implementations of board and commlink
        board_.reset(new AirSimSimpleFlightBoard(&params_));
        comm_link_.reset(new AirSimSimpleFlightCommLink());
        estimator_.reset(new AirSimSimpleFlightEstimator());

        //create firmware
        firmware_.reset(new simple_flight::Firmware(&params_, board_.get(), comm_link_.get(), estimator_.get()));
    }


public: //VehicleApiBase implementation
    virtual void resetImplementation() override
    {
        MultirotorApiBase::resetImplementation();

        firmware_->reset();
    }
    virtual void update() override
    {
        MultirotorApiBase::update();

        //update controller which will update actuator control signal
        firmware_->update();
    }
    virtual bool isApiControlEnabled() const override
    {
        return firmware_->offboardApi().hasApiControl();
    }
    virtual void enableApiControl(bool is_enabled) override
    {
        if (is_enabled) {
            //comm_link should print message so no extra handling for errors
            std::string message;
            firmware_->offboardApi().requestApiControl(message);
        }
        else
            firmware_->offboardApi().releaseApiControl();
    }
    virtual bool armDisarm(bool arm) override
    {
        std::string message;
        if (arm)
            return firmware_->offboardApi().arm(message);
        else
            return firmware_->offboardApi().disarm(message);
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getHomeGeoPoint());
    }
    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        comm_link_->getStatusMessages(messages);
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public: //MultirotorApiBase implementation
    virtual real_T getActuation(unsigned int rotor_index) const override
    {
        auto control_signal = board_->getMotorControlSignal(rotor_index);
        return control_signal;
    }
    virtual size_t getActuatorCount() const override
    {
        return vehicle_params_->getParams().rotor_count;
    }
    virtual void moveByRC(const RCData& rc_data) override
    {
        setRCData(rc_data);
    }
    virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
    {
        board_->setGroundTruthKinematics(kinematics);
        estimator_->setGroundTruthKinematics(kinematics, environment);
    }
    virtual bool setRCData(const RCData& rc_data) override
    {
        last_rcData_ = rc_data;
        if (rc_data.is_valid) {
            board_->setIsRcConnected(true);
            board_->setInputChannel(0, rc_data.roll); //X
            board_->setInputChannel(1, rc_data.yaw); //Y
            board_->setInputChannel(2, rc_data.throttle); //F
            board_->setInputChannel(3, -rc_data.pitch); //Z
            board_->setInputChannel(4, static_cast<float>(rc_data.getSwitch(0))); //angle rate or level
            board_->setInputChannel(5, static_cast<float>(rc_data.getSwitch(1))); //Allow API control
            board_->setInputChannel(6, static_cast<float>(rc_data.getSwitch(2)));
            board_->setInputChannel(7, static_cast<float>(rc_data.getSwitch(3)));
            board_->setInputChannel(8, static_cast<float>(rc_data.getSwitch(4)));
            board_->setInputChannel(9, static_cast<float>(rc_data.getSwitch(5)));
            board_->setInputChannel(10, static_cast<float>(rc_data.getSwitch(6)));
            board_->setInputChannel(11, static_cast<float>(rc_data.getSwitch(7)));
        }
        else { //else we don't have RC data
            board_->setIsRcConnected(false);
        }

        return true;
    }

protected: 
    virtual Kinematics::State getKinematicsEstimated() const override
    {
        return AirSimSimpleFlightCommon::toKinematicsState3r(firmware_->offboardApi().
            getStateEstimator().getKinematicsEstimated());
    }

    virtual Vector3r getPosition() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getPosition();
        return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Vector3r getVelocity() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getLinearVelocity();
        return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Quaternionr getOrientation() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getOrientation();
        return AirSimSimpleFlightCommon::toQuaternion(val);
    }

    virtual LandedState getLandedState() const override
    {
        return firmware_->offboardApi().getLandedState() ? LandedState::Landed : LandedState::Flying;
    }

    virtual RCData getRCData() const override
    {
        //return what we received last time through setRCData
        return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
        return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getGeoPoint());
    }

    virtual float getCommandPeriod() const override
    {
        return 1.0f / 50; //50hz
    }

    virtual float getTakeoffZ() const override
    {
        // pick a number, 3 meters is probably safe 
        // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
        return params_.takeoff.takeoff_z;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
    }

    virtual void commandRollPitchThrottle(float pitch, float roll, float throttle, float yaw_rate) override
    {
        //Utils::log(Utils::stringf("commandRollPitchThrottle %f, %f, %f, %f", pitch, roll, throttle, yaw_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleRate, GoalModeType::Passthrough);

        simple_flight::Axis4r goal(roll, pitch, yaw_rate, throttle);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        //Utils::log(Utils::stringf("commandRollPitchZ %f, %f, %f, %f", pitch, roll, z, yaw));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(roll, pitch, yaw, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        //Utils::log(Utils::stringf("commandVelocity %f, %f, %f, %f", vx, vy, vz, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld, 
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, 
            GoalModeType::VelocityWorld);

        simple_flight::Axis4r goal(vy, vx, Utils::degreesToRadians(yaw_mode.yaw_or_rate), vz);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        //Utils::log(Utils::stringf("commandVelocityZ %f, %f, %f, %f", vx, vy, z, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld, 
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, 
            GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(vy, vx, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        //Utils::log(Utils::stringf("commandPosition %f, %f, %f, %f", x, y, z, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::PositionWorld, GoalModeType::PositionWorld, 
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, 
            GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(y, x, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual const MultirotorApiParams& getMultirotorApiParams() const override
    {
        return safety_params_;
    }

    //*** End: MultirotorApiBase implementation ***//

private:
    //convert pitch, roll, yaw from -1 to 1 to PWM
    static uint16_t angleToPwm(float angle)
    {
        return static_cast<uint16_t>(angle * 500.0f + 1500.0f);
    }
    static uint16_t thrustToPwm(float thrust)
    {
        return static_cast<uint16_t>((thrust < 0 ? 0 : thrust) * 1000.0f + 1000.0f);
    }
    static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

    void readSettings(const AirSimSettings::VehicleSetting& vehicle_setting)
    {
        params_.default_vehicle_state = simple_flight::VehicleState::fromString(
            vehicle_setting.default_vehicle_state == "" ? "Armed" : vehicle_setting.default_vehicle_state);

        remote_control_id_ = vehicle_setting.rc.remote_control_id;
        params_.rc.allow_api_when_disconnected = vehicle_setting.rc.allow_api_when_disconnected;
        params_.rc.allow_api_always = vehicle_setting.allow_api_always;
    }

private:
    const MultiRotorParams* vehicle_params_;

    int remote_control_id_ = 0;
    simple_flight::Params params_;

    unique_ptr<AirSimSimpleFlightBoard> board_;
    unique_ptr<AirSimSimpleFlightCommLink> comm_link_;
    unique_ptr<AirSimSimpleFlightEstimator> estimator_;
    unique_ptr<simple_flight::IFirmware> firmware_;

    MultirotorApiParams safety_params_;

    RCData last_rcData_;
};

}} //namespace
#endif 
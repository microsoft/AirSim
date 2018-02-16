// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleFlightDroneController_hpp
#define msr_airlib_SimpleFlightDroneController_hpp

#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
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
#include "common/AirSimSettings.hpp"


namespace msr { namespace airlib {

class SimpleFlightDroneController : public DroneControllerBase {

public:
    SimpleFlightDroneController(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSettings& vehicle_settings)
        : vehicle_params_(vehicle_params)
    {
        readSettings(vehicle_settings);

        //TODO: set below properly for better high speed safety
        safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;

        //create sim implementations of board and commlink
        board_.reset(new AirSimSimpleFlightBoard(&params_));
        comm_link_.reset(new AirSimSimpleFlightCommLink());
        estimator_.reset(new AirSimSimpleFlightEstimator());

        //create firmware
        firmware_.reset(new simple_flight::Firmware(&params_, board_.get(), comm_link_.get(), estimator_.get()));
    }

    void setGroundTruth(PhysicsBody* physics_body) override
    {
        physics_body_ = physics_body;

        board_->setKinematics(& physics_body_->getKinematics());
        estimator_->setKinematics(& physics_body_->getKinematics(), & physics_body_->getEnvironment());
    }

public:
    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override
    {
        DroneControllerBase::reset();

        firmware_->reset();
    }

    virtual void update() override
    {
        DroneControllerBase::update();

        firmware_->update();
    }

    virtual size_t getVertexCount() override
    {
        return vehicle_params_->getParams().rotor_count;
    }

    virtual bool isAvailable(std::string& message) override
    {
        unused(message);
        return true;
    }

    virtual real_T getVertexControlSignal(unsigned int rotor_index) override
    {
        auto control_signal = board_->getMotorControlSignal(rotor_index);
        return control_signal;
    }

    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        comm_link_->getStatusMessages(messages);
    }

    virtual bool isApiControlEnabled() override
    {
        return firmware_->offboardApi().hasApiControl();
    }

    virtual bool isSimulationMode() override
    {
        //TODO: after we get real board implementation, change this
        return true;
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
    
    virtual void setSimulationMode(bool is_set) override
    {
        if (!is_set)
            throw VehicleCommandNotImplementedException("setting non-simulation mode is not supported yet");
    }
    //*** End: VehicleControllerBase implementation ***//

//*** Start: DroneControllerBase implementation ***//
public:
    virtual Kinematics::State getKinematicsEstimated() override
    {
        return AirSimSimpleFlightCommon::toKinematicsState3r(firmware_->offboardApi().
            getStateEstimator().getKinematicsEstimated());
    }

    virtual Vector3r getPosition() override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getPosition();
        return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Vector3r getVelocity() override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getLinearVelocity();
        return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Quaternionr getOrientation() override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getOrientation();
        return AirSimSimpleFlightCommon::toQuaternion(val);    
    }

    virtual LandedState getLandedState() override
    {
        //TODO: implement this
        return LandedState::Landed;
    }

    virtual int getRemoteControlID()  override
    { 
        return remote_control_id_;
    }
    
    virtual RCData getRCData() override
    {
        return last_rcData_;
    }

    virtual void setRCData(const RCData& rcData) override
    {
        last_rcData_ = rcData;
        if (rcData.is_valid) {
            board_->setIsRcConnected(true);
            board_->setInputChannel(0, rcData.roll); //X
            board_->setInputChannel(1, rcData.yaw); //Y
            board_->setInputChannel(2, rcData.throttle); //F
            board_->setInputChannel(3, -rcData.pitch); //Z
            board_->setInputChannel(4, static_cast<float>(rcData.switch1));
            board_->setInputChannel(5, static_cast<float>(rcData.switch2));
            board_->setInputChannel(6, static_cast<float>(rcData.switch3));
            board_->setInputChannel(7, static_cast<float>(rcData.switch4));
            board_->setInputChannel(8, static_cast<float>(rcData.switch5)); 
            board_->setInputChannel(9, static_cast<float>(rcData.switch6)); 
            board_->setInputChannel(10, static_cast<float>(rcData.switch7)); 
            board_->setInputChannel(11, static_cast<float>(rcData.switch8)); 
        }
        else { //else we don't have RC data
            board_->setIsRcConnected(false);
        }
    }

    virtual bool armDisarm(bool arm, CancelableBase& cancelable_action) override
    {
        unused(cancelable_action);

        std::string message;
        if (arm)
            return firmware_->offboardApi().arm(message);
        else
            return firmware_->offboardApi().disarm(message);
    }

    virtual GeoPoint getHomeGeoPoint() override
    {
        return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getHomeGeoPoint());
    }

    virtual GeoPoint getGpsLocation() override
    {
        return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getGeoPoint());
    }

    virtual void reportTelemetry(float renderTime) override
    {
        unused(renderTime);
        //TODO: implement this
    }

    virtual float getCommandPeriod() override
    {
        return 1.0f/50; //50hz
    }

    virtual float getTakeoffZ() override
    {
        // pick a number, 3 meters is probably safe 
        // enough to get out of the backwash turbulance.  Negative due to NED coordinate system.
        return params_.takeoff.takeoff_z;
    }

    virtual float getDistanceAccuracy() override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
    }

protected: 
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        Utils::log(Utils::stringf("commandRollPitchZ %f, %f, %f, %f", pitch, roll, z, yaw));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(roll, pitch, yaw, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        Utils::log(Utils::stringf("commandVelocity %f, %f, %f, %f", vx, vy, vz, yaw_mode.yaw_or_rate));

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
        Utils::log(Utils::stringf("commandVelocityZ %f, %f, %f, %f", vx, vy, z, yaw_mode.yaw_or_rate));

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
        Utils::log(Utils::stringf("commandPosition %f, %f, %f, %f", x, y, z, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::PositionWorld, GoalModeType::PositionWorld, 
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel, 
            GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(y, x, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual const VehicleParams& getVehicleParams() override
    {
        return safety_params_;
    }

    //*** End: DroneControllerBase implementation ***//

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

    void readSettings(const AirSimSettings::VehicleSettings& vehicle_settings)
    {
        //find out which RC we should use
        Settings simple_flight_settings;
        vehicle_settings.getRawSettings(simple_flight_settings);
        params_.default_vehicle_state = simple_flight::VehicleState::fromString(
            simple_flight_settings.getString("DefaultVehicleState", "Armed")); //Inactive, Armed

        Settings rc_settings;
        simple_flight_settings.getChild("RC", rc_settings);
        remote_control_id_ = rc_settings.getInt("RemoteControlID", 0);
        params_.rc.allow_api_when_disconnected = 
            rc_settings.getBool("AllowAPIWhenDisconnected", false);
        params_.rc.allow_api_always = 
            rc_settings.getBool("AllowAPIAlways", true);
    }

private:
    const MultiRotorParams* vehicle_params_;
    PhysicsBody* physics_body_;

    int remote_control_id_ = 0;
    simple_flight::Params params_;

    unique_ptr<AirSimSimpleFlightBoard> board_;
    unique_ptr<AirSimSimpleFlightCommLink> comm_link_;
    unique_ptr<AirSimSimpleFlightEstimator> estimator_;
    unique_ptr<simple_flight::IFirmware> firmware_;

    VehicleParams safety_params_;

    RCData last_rcData_;
};

}} //namespace
#endif 
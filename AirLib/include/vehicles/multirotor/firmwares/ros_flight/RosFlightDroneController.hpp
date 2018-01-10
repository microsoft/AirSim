// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RosFlightDroneController_hpp
#define msr_airlib_RosFlightDroneController_hpp

#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "AirSimRosFlightBoard.hpp"
#include "AirSimRosFlightCommLink.hpp"
#include "common/AirSimSettings.hpp"

STRICT_MODE_OFF
#include "firmware/firmware.hpp"
STRICT_MODE_ON

namespace msr { namespace airlib {

class RosFlightDroneController : public DroneControllerBase {

public:
    RosFlightDroneController(const SensorCollection* sensors, const MultiRotorParams* vehicle_params)
        : vehicle_params_(vehicle_params)
    {
        sensors_ = sensors;

        board_.reset(new AirSimRosFlightBoard(&vehicle_params_->getParams().enabled_sensors, sensors_));
        comm_link_.reset(new AirSimRosFlightCommLink());
        firmware_.reset(new ros_flight::Firmware(board_.get(), comm_link_.get()));
        firmware_->setup();

        Settings child;
        Settings::singleton().getChild("RosFlight", child);
        remote_control_id_ = child.getInt("RemoteControlID", 0);
    }

    void setGroundTruth(PhysicsBody* physics_body) override
    {
        environment_ = & physics_body->getEnvironment();
        kinematics_ = & physics_body->getKinematics();
    }

public:
    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override
    {
        DroneControllerBase::reset();

        board_->system_reset(false);
    }

    virtual void update() override
    {
        DroneControllerBase::update();

        board_->notifySensorUpdated(ros_flight::Board::SensorType::Imu);
        firmware_->loop();
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
        //convert counter clockwise index to ArduCopter's QuadX style index
        unsigned int index_quadx;
        switch (rotor_index)
        {
        case 0: index_quadx = 1; break;
        case 1: index_quadx = 2; break;
        case 2: index_quadx = 3; break;
        case 3: index_quadx = 0; break;
        default:
            throw std::runtime_error("Rotor index beyond 3 is not supported yet in ROSFlight firmware");
        }

        auto control_signal = board_->getMotorControlSignal(index_quadx);

        return control_signal;
    }

    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        comm_link_->getStatusMessages(messages);
    }

    virtual bool isApiControlEnabled() override
    {
        //TODO: support offboard mode
        return false;
    }

    virtual bool isSimulationMode() override
    {
        return true;
    }

    virtual void enableApiControl(bool is_enabled) override
    {
        //TODO: implement this
        unused(is_enabled);
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
        return *kinematics_;
    }
    
    virtual Vector3r getPosition() override
    {
        return kinematics_->pose.position;
    }

    virtual Vector3r getVelocity() override
    {
        return kinematics_->twist.linear;
    }

    virtual Quaternionr getOrientation() override
    {
        return kinematics_->pose.orientation;
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
        return RCData();
    }

    virtual void setRCData(const RCData& rcData) override
    {
        if (rcData.is_valid) {
            board_->setInputChannel(0, angleToPwm(rcData.roll)); //X
            board_->setInputChannel(1, angleToPwm(rcData.yaw)); //Y
            board_->setInputChannel(2, thrustToPwm(rcData.throttle)); //F
            board_->setInputChannel(3, angleToPwm(-rcData.pitch)); //Z
            board_->setInputChannel(4, switchToPwm(rcData.switch1));
            board_->setInputChannel(5, switchToPwm(rcData.switch2));
            board_->setInputChannel(6, switchToPwm(rcData.switch3));
            board_->setInputChannel(7, switchToPwm(rcData.switch4));
            board_->setInputChannel(8, switchToPwm(rcData.switch5)); 
            board_->setInputChannel(9, switchToPwm(rcData.switch6)); 
            board_->setInputChannel(10, switchToPwm(rcData.switch7)); 
            board_->setInputChannel(11, switchToPwm(rcData.switch8)); 
        }
        //else we don't have RC data
    }

    virtual bool armDisarm(bool arm, CancelableBase& cancelable_action) override
    {
        unused(arm);
        unused(cancelable_action);

        return true;
    }

    virtual bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override
    {
        unused(max_wait_seconds);
        unused(cancelable_action);

        return true;
    }

    virtual bool land(float max_wait_seconds, CancelableBase& cancelable_action) override
    {
        unused(max_wait_seconds);
        unused(cancelable_action);

        return true;
    }

    virtual bool goHome(CancelableBase& cancelable_action) override
    {
        unused(cancelable_action);
        return true;
    }

    virtual bool hover(CancelableBase& cancelable_action) override
    {
        unused(cancelable_action);
        return true;
    }

    virtual GeoPoint getHomeGeoPoint() override
    {
        return environment_->getInitialState().geo_point;
    }

    virtual GeoPoint getGpsLocation() override
    {
        return environment_->getState().geo_point;
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
        return -3.0f;  
    }

    virtual float getDistanceAccuracy() override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
    }

protected: 
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        unused(pitch);
        unused(roll);
        unused(z);
        unused(yaw);

        //TODO: implement this
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        unused(vx);
        unused(vy);
        unused(vz);
        unused(yaw_mode);

        //TODO: implement this
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        unused(vx);
        unused(vy);
        unused(z);
        unused(yaw_mode);

        //TODO: implement this
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        unused(x);
        unused(y);
        unused(z);
        unused(yaw_mode);

        //TODO: implement this
    }

    virtual const VehicleParams& getVehicleParams() override
    {
        //used for safety algos. For now just use defaults
        static const VehicleParams safety_params;
        return safety_params;
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
    static uint16_t switchToPwm(uint switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

private:
    const MultiRotorParams* vehicle_params_;
    const Kinematics::State* kinematics_;
    const Environment* environment_;
    const SensorCollection* sensors_;

    int remote_control_id_ = 0;

    unique_ptr<AirSimRosFlightBoard> board_;
    unique_ptr<AirSimRosFlightCommLink> comm_link_;
    unique_ptr<ros_flight::Firmware> firmware_;
};

}} //namespace
#endif 
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleFlightDroneController_hpp
#define msr_airlib_SimpleFlightDroneController_hpp

#include "controllers/DroneControllerBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "AirSimSimpleFlightBoard.hpp"
#include "AirSimSimpleFlightCommLink.hpp"
#include "controllers/Settings.hpp"
#include "firmware/Firmware.hpp"


namespace msr { namespace airlib {

class SimpleFlightDroneController : public DroneControllerBase {

public:
    SimpleFlightDroneController(const MultiRotorParams* vehicle_params)
        : vehicle_params_(vehicle_params)
    {
        //create sim implementations of board and commlink
        board_.reset(new AirSimSimpleFlightBoard(&params_));
        comm_link_.reset(new AirSimSimpleFlightCommLink());

        //create firmware
        firmware_.reset(new simple_flight::Firmware(board_.get(), comm_link_.get(), &params_));
        firmware_->reset();

        //find out which RC we should use
        Settings child;
        Settings::singleton().getChild("SimpleFlight", child);
        remote_control_id_ = child.getInt("RemoteControlID", 0);
    }

    void initializePhysics(const Environment* environment, const Kinematics::State* kinematics)
    {
        environment_ = environment;
        kinematics_ = kinematics;
        board_->setKinematics(kinematics_);
    }

public:
    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override
    {
        firmware_->reset();
    }

    virtual void update() override
    {
        firmware_->update();
    }

    virtual void start() override
    {
    }

    virtual void stop() override
    {
    }

    virtual size_t getVertexCount() override
    {
        return vehicle_params_->getParams().rotor_count;
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

    virtual bool isOffboardMode() override
    {
        //TODO: support offboard mode
        return false;
    }

    virtual bool isSimulationMode() override
    {
        return true;
    }

    virtual void setOffboardMode(bool is_set) override
    {
        //TODO: implement this
    }

    virtual void setSimulationMode(bool is_set) override
    {
        if (!is_set)
            throw VehicleCommandNotImplementedException("setting non-simulation mode is not supported yet");
    }
    //*** End: VehicleControllerBase implementation ***//

//*** Start: DroneControllerBase implementation ***//
public:
    Vector3r getPosition() override
    {
        return kinematics_->pose.position;
    }

    Vector3r getVelocity() override
    {
        return kinematics_->twist.linear;
    }

    Quaternionr getOrientation() override
    {
        return kinematics_->pose.orientation;
    }

    LandedState getLandedState() override
    {
        //TODO: implement this
        return LandedState::Landed;
    }

    virtual int getRemoteControlID()  override
    { 
        return remote_control_id_;
    }
    
    RCData getRCData() override
    {
        return RCData();
    }

    void setRCData(const RCData& rcData) override
    {
        if (rcData.is_connected) {
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
        //else we don't have RC data
    }

    bool armDisarm(bool arm, CancelableBase& cancelable_action) override
    {
        return true;
    }

    bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override
    {
        return true;
    }

    bool land(float max_wait_seconds, CancelableBase& cancelable_action) override
    {
        return true;
    }

    bool goHome(CancelableBase& cancelable_action) override
    {
        return true;
    }

    bool hover(CancelableBase& cancelable_action) override
    {
        return true;
    }

    GeoPoint getHomePoint() override
    {
        return environment_->getInitialState().geo_point;
    }

    GeoPoint getGpsLocation() override
    {
        return environment_->getState().geo_point;
    }

    virtual void reportTelemetry(float renderTime) override
    {
        //TODO: implement this
    }

    float getCommandPeriod() override
    {
        return 1.0f/50; //50hz
    }

    float getTakeoffZ() override
    {
        // pick a number, 3 meters is probably safe 
        // enough to get out of the backwash turbulance.  Negative due to NED coordinate system.
        return -3.0f;  
    }

    float getDistanceAccuracy() override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance travelled
    }

protected: 
    void commandRollPitchZ(float pitch, float roll, float z, float yaw) override
    {
        //TODO: implement this
    }

    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        //TODO: implement this
    }

    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        //TODO: implement this
    }

    void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        //TODO: implement this
    }

    const VehicleParams& getVehicleParams() override
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
    static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

private:
    const MultiRotorParams* vehicle_params_;
    const Kinematics::State* kinematics_;
    const Environment* environment_;

    int remote_control_id_ = 0;
    simple_flight::Params params_;

    unique_ptr<AirSimSimpleFlightBoard> board_;
    unique_ptr<AirSimSimpleFlightCommLink> comm_link_;
    unique_ptr<simple_flight::Firmware> firmware_;
};

}} //namespace
#endif 
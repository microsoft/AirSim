// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MavLinkDroneControl_hpp
#define air_MavLinkDroneControl_hpp

#include "control/DroneControlBase.hpp"
#include "common/Common.hpp"
#include "MavLinkVehicle.hpp"
#include "MavLinkConnection.hpp"
#include "PidController.hpp"

namespace msr { namespace airlib {
   
/// This class uses AirSim/MavLinkCom library to talk to a mavlink connected drone.
/// (like the Pixhawk based DIY drones).  See https://github.com/mavlink/mavlink.
class MavLinkDroneControl : public DroneControlBase {
public:
    //TODO: create common struct for below
    struct Parameters {
        //default setup is for connecting to UDP via QGC port
        string connectionType = "udp";
        int system_id = 135;
        int component_id = 1;
        string udpAddress = "127.0.0.1";
        double udpPortDouble = 14550;
        int udpPort = static_cast<int>(udpPortDouble);
        string serialDevice = "COM4";
        double baudRateDouble = 115200;
        int baudRate = static_cast<int>(baudRateDouble);
    };

    MavLinkDroneControl(const Parameters& params);
	MavLinkDroneControl(int local_system_id, int local_component_id, std::shared_ptr<mavlinkcom::MavLinkConnection> connection);

    Vector3r getPosition() override;
    Vector3r getVelocity() override;
    Quaternionr getOrientation() override;
    RCData getRCData() override;
    bool isOffboardMode();
    double timestampNow() override;
    
    bool armDisarm(bool arm, CancelableActionBase& cancelable_action) override;
    bool requestControl(CancelableActionBase& cancelable_action) override;
    bool releaseControl(CancelableActionBase& cancelable_action) override;
    bool takeoff(float max_wait_seconds, CancelableActionBase& cancelable_action) override;
    bool land(CancelableActionBase& cancelable_action) override;
    bool goHome(CancelableActionBase& cancelable_action) override; 
    bool hover(CancelableActionBase& cancelable_action) override;
    GeoPoint getHomePoint() override;
    GeoPoint getGpsLocation() override;

    float getCommandPeriod() override;
    float getTakeoffZ() override;
    float getDistanceAccuracy() override;

protected:  //keep low level commands hidden from outside as they have no safety checks
    void commandRollPitchZ(float pitch, float roll, float z, float yaw) override;
    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override;
    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override;
    void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override;
    void commandVirtualRC(const RCData& rc_data) override;
    void commandEnableVirtualRC(bool enable) override;
    const VehicleParams& getVehicleParams() override;

private:
	static const VehicleParams& getInternalVehicleParams();
    static shared_ptr<mavlinkcom::MavLinkConnection> createConnection(const Parameters& params);
    bool validateRCData(const RCData& rc_data);

	std::shared_ptr<mavlinkcom::MavLinkVehicle> drone_;
	int state_version;
	mavlinkcom::VehicleState current_state;
	void updateState();
    float target_height_ = 0;
    bool is_offboard_mode_;
    PidController thrust_controller_;

    Parameters params_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ros_MavLinkDroneController_hpp
#define air_ros_MavLinkDroneController_hpp

#include "common/CommonStructs.hpp"
#include "common/Common.hpp"
#include "common/VectorMath.hpp"
#include "vehicles/MultiRotor.hpp"
#include "controllers/DroneControllerBase.hpp"
#include <queue>
#include <mutex>
#include <string>
#include <vector>

namespace msr { namespace airlib {


class MavLinkDroneController : public DroneControllerBase
{
public:
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::MultiRotor MultiRotor;

    struct ConnectionInfo {
        /* Default values are requires so uninitialized instance doesn't have random values */

        std::string vehicle_name = "Pixhawk";
        bool use_serial = true; // false means use UDP instead
        //Used to connect via HITL: needed only if use_serial = true
        std::string serial_port = "*";
        int baud_rate = 115200;

        //Used to connect to drone over UDP: needed only if use_serial = false
        std::string ip_address = "127.0.0.1";
        int ip_port = 14560;

		// The PX4 SITL app requires receiving drone commands over a different mavlink channel.
		// So set this to '127.0.0.1' to enable that separate command channel.
		std::string sitl_ip_address = "";
		int sitl_ip_port = 14556;

        // The log viewer can be on a different machine, so you can configure it's ip address and port here.
        int logviewer_ip_port = 14388;
        std::string logviewer_ip_address = "127.0.0.1";

        // The QGroundControl app can be on a different machine, so you can configure it's ip address and port here.
        int qgc_ip_port = 14550;
        std::string qgc_ip_address = "127.0.0.1";

        // mavlink vehicle identifiers
        uint8_t sim_sysid = 142;
        int sim_compid = 42;
        uint8_t offboard_sysid = 134;
        int offboard_compid = 1;
        uint8_t vehicle_sysid = 135;
        int vehicle_compid = 1;    

        // if you want to select a specific local network adapter so you can reach certain remote machines (e.g. wifi versus ethernet) 
        // then you will want to change the LocalHostIp accordingly.  This default only works when log viewer and QGC are also on the
        // same machine.  Whatever network you choose it has to be the same one for external
        std::string local_host_ip = "127.0.0.1";
    };

public:
    //required for pimpl
    MavLinkDroneController();
    ~MavLinkDroneController();

    //non-base interface specific to MavLinKDroneController
    void initialize(const ConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation);
    ConnectionInfo getMavConnectionInfo();
    static std::string findPixhawk();

    //TODO: get rid of below methods?
    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height);
    void getMocapPose(Vector3r& position, Quaternionr& orientation);
    void sendMocapPose(const Vector3r& position, const Quaternionr& orientation);
    bool hasVideoRequest();

    //*** Start: VehicleControllerBase implementation ***//
    virtual void reset() override;
    virtual void update(real_T dt) override;
    virtual void start() override;
    virtual void stop() override;
    virtual size_t getVertexCount() override;
    virtual real_T getVertexControlSignal(unsigned int rotor_index) override;
    virtual void getStatusMessages(std::vector<std::string>& messages) override;

    virtual bool isOffboardMode() override;
    virtual bool isSimulationMode() override;
    virtual void setOffboardMode(bool is_set) override;
    virtual void setSimulationMode(bool is_set) override;
    virtual void setUserInputs(const vector<float>& inputs) override;
    //*** End: VehicleControllerBase implementation ***//


    //*** Start: DroneControllerBase implementation ***//
public:
    Vector3r getPosition() override;
    Vector3r getVelocity() override;
    Quaternionr getOrientation() override;
    RCData getRCData() override;
    double timestampNow() override;

    bool armDisarm(bool arm, CancelableBase& cancelable_action) override;
    bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action) override;
    bool land(CancelableBase& cancelable_action) override;
    bool goHome(CancelableBase& cancelable_action) override; 
    bool hover(CancelableBase& cancelable_action) override;
    GeoPoint getHomePoint() override;
    GeoPoint getGpsLocation() override;
	virtual void reportTelemetry(float renderTime) override;

    float getCommandPeriod() override;
    float getTakeoffZ() override;
    float getDistanceAccuracy() override;
protected: 
    void commandRollPitchZ(float pitch, float roll, float z, float yaw) override;
    void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override;
    void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override;
    void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override;
    void commandVirtualRC(const RCData& rc_data) override;
    void commandEnableVirtualRC(bool enable) override;
    const VehicleParams& getVehicleParams() override;
    //*** End: DroneControllerBase implementation ***//

public: //pimpl
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

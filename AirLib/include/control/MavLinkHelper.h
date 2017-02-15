// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ros_MavLinkHelper_hpp
#define air_ros_MavLinkHelper_hpp

#include "common/CommonStructs.hpp"
#include "common/Common.hpp"
#include "common/VectorMath.hpp"
#include "vehicles/controllers/ControllerBase.hpp"
#include "vehicles/MultiRotor.hpp"
#include "DroneControlBase.hpp"
#include <queue>
#include <mutex>
#include <string>
#include <vector>


namespace msr { namespace airlib {


class MavLinkHelper : public ControllerBase
{
public:
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::MultiRotor MultiRotor;

    //required for pimpl
    MavLinkHelper();
    ~MavLinkHelper();


    struct HILConnectionInfo {

        std::string vehicle_name;

        bool use_serial; // false means use UDP instead

        //needed only if use_serial = false
        std::string ip_address;
        int ip_port;

        //needed only if use_serial = true
        std::string serial_port;
        int baud_rate;

    };

    int getRotorControlsCount();
    void connectToExternalSim();
    void connectToHIL(const HILConnectionInfo& connection_info);
    void connectToVideoServer();    
    bool connectToLogViewer();
    bool connectToQGC();
    void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height);
    void getMocapPose(Vector3r& position, Quaternionr& orientation);
    void sendMocapPose(const Vector3r& position, const Quaternionr& orientation);
    void sendCollison(float normalX, float normalY, float normalZ);
    bool hasVideoRequest();
    void sendHILSensor(const Vector3r& acceleration, const Vector3r& gyro, const Vector3r& mag, float abs_pressure, float pressure_alt);
    void sendHILGps(const GeoPoint& geo_point, const Vector3r& velocity, float velocity_xy, float cog, float eph, float epv, int fix_type, unsigned int satellites_visible);
    void getStatusMessages(std::vector<std::string>& messages);
    void close();
    void setNormalMode();
    void setHILMode();
    std::string findPixhawk();
    DroneControlBase* createOrGetDroneControl();

    void initialize(const MultiRotor* vehicle);
    virtual real_T getRotorControlSignal(unsigned int rotor_index) override;
    //*** Start: UpdatableState implementation ***//
    virtual void reset() override;
    virtual void update(real_T dt) override;
    //*** End: UpdatableState implementation ***//

    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

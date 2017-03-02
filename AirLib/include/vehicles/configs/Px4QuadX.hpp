// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Pix4QuadX_hpp
#define msr_airlib_vehicles_Pix4QuadX_hpp

#include "vehicles/MultiRotorParams.hpp"
#include "controllers/MavLinkDroneController.hpp"
#include "controllers/Settings.hpp"

//sensors
#include "sensors/barometer/BarometerSimple.hpp"
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"

namespace msr { namespace airlib {

class Px4QuadX : public MultiRotorParams {
public:
    Px4QuadX(const string& connection_name = string("Pixhawk"))
        : connection_name_(connection_name)
    {
    }

protected:
    virtual void setup(Params& params, SensorCollection& sensors, unique_ptr<DroneControllerBase>& controller) override
    {
        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

        //set up mass
        params.mass = 1.0f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        //set up dimensions of core body box
        params.body_box.x = 0.180f; params.body_box.y = 0.11f; params.body_box.z = 0.040f;
        real_T rotor_z = 2.5f / 100;

        //computer rotor poses
        initializeRotorPoses(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);
        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        //create sensors
        createStandardSensors(sensors, params.enabled_sensors);
        //create MavLink controller for PX4
        createMavController(controller, sensors);

        //leave everything else to defaults
    }

private:
    void createMavController(unique_ptr<DroneControllerBase>& controller, SensorCollection& sensors)
    {
        controller.reset(new MavLinkDroneController());
        auto mav_controller = static_cast<MavLinkDroneController*>(controller.get());
        mav_controller->initialize(getConnectionInfo(), &sensors, true);
    }

    MavLinkDroneController::ConnectionInfo getConnectionInfo()
    {
        //start with defaults
        MavLinkDroneController::ConnectionInfo connection_info;
        connection_info.vehicle_name = connection_name_;

        //read settings and override defaults
        Settings& settings = Settings::singleton();
        auto settings_filename = Settings::singleton().getFileName();
        if (!settings_filename.empty()) {
            Settings child = settings.getChild(connection_info.vehicle_name);
            // allow json overrides on a per-vehicle basis.
            connection_info.sim_sysid = static_cast<uint8_t>(child.getInt("SimSysID", connection_info.sim_sysid));
            connection_info.sim_compid = child.getInt("SimCompID", connection_info.sim_compid);

            connection_info.vehicle_sysid = static_cast<uint8_t>(child.getInt("VehicleSysID", connection_info.vehicle_sysid));
            connection_info.vehicle_compid = child.getInt("VehicleCompID", connection_info.vehicle_compid);

            connection_info.offboard_sysid = static_cast<uint8_t>(child.getInt("OffboardSysID", connection_info.offboard_sysid));
            connection_info.offboard_compid = child.getInt("OffboardCompID", connection_info.offboard_compid);

            connection_info.logviewer_ip_address = child.getString("LogViewerHostIp", connection_info.logviewer_ip_address);
            connection_info.logviewer_ip_port = child.getInt("LogViewerPort", connection_info.logviewer_ip_port);

            connection_info.qgc_ip_address = child.getString("QgcHostIp", connection_info.qgc_ip_address);
            connection_info.qgc_ip_port = child.getInt("QgcPort", connection_info.qgc_ip_port);

            connection_info.ip_address = child.getString("SitlIp", connection_info.ip_address);
            connection_info.ip_port = child.getInt("SitlPort", connection_info.ip_port);

            connection_info.local_host_ip = child.getString("LocalHostIp", connection_info.local_host_ip);


            connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);
            connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
            connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
            connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
            connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);
        }

        //if settings didn't exist then create
        if (connection_info.vehicle_name.size() > 0 && !Settings::singleton().isLoadSuccess()) {
            Settings child;
            child.setInt("SimSysID", connection_info.sim_sysid);
            child.setInt("SimCompID", connection_info.sim_compid);

            child.setInt("VehicleSysID", connection_info.vehicle_sysid);
            child.setInt("VehicleCompID", connection_info.vehicle_compid);

            child.setInt("OffboardSysID", connection_info.offboard_sysid);
            child.setInt("OffboardCompID", connection_info.offboard_compid);

            child.setString("LogViewerHostIp", connection_info.logviewer_ip_address);
            child.setInt("LogViewerPort", connection_info.logviewer_ip_port);

            child.setString("QgcHostIp", connection_info.qgc_ip_address);
            child.setInt("QgcPort", connection_info.qgc_ip_port);

            child.setString("SitlIp", connection_info.ip_address);
            child.setInt("SitlPort", connection_info.ip_port);

            child.setString("LocalHostIp", connection_info.local_host_ip);


            child.setBool("UseSerial", connection_info.use_serial);
            child.setString("UdpIp", connection_info.ip_address);
            child.setInt("UdpPort", connection_info.ip_port);
            child.setString("SerialPort", connection_info.serial_port);
            child.setInt("SerialBaudRate", connection_info.baud_rate);
            settings.setChild(connection_info.vehicle_name, child);
            settings.saveJSonFile(L"settings.json");
        }

        return connection_info;
    }

    void createStandardSensors(SensorCollection& sensors, const EnabledSensors& enabled_sensors)
    {
        sensor_storage_.clear();
        if (enabled_sensors.imu)
            sensors.insert(createSensor<ImuSimple>(), SensorCollection::SensorType::Imu);
        if (enabled_sensors.magnetometer)
            sensors.insert(createSensor<MagnetometerSimple>(), SensorCollection::SensorType::Magnetometer);
        if (enabled_sensors.gps)
            sensors.insert(createSensor<GpsSimple>(), SensorCollection::SensorType::Gps);
        if (enabled_sensors.barometer)
            sensors.insert(createSensor<BarometerSimple>(), SensorCollection::SensorType::Barometer);
    }

    template<typename SensorClass>
    SensorBase* createSensor()
    {
        sensor_storage_.emplace_back(unique_ptr<SensorClass>(new SensorClass()));
        return sensor_storage_.back().get();
    }

private:
    vector<unique_ptr<SensorBase>> sensor_storage_;
    string connection_name_;
};

}} //namespace
#endif

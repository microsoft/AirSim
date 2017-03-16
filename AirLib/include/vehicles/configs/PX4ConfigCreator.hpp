// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_PX4ConfigCreator_hpp
#define msr_airlib_vehicles_PX4ConfigCreator_hpp

#include "controllers/MavLinkDroneController.hpp"
#include "controllers/Settings.hpp"
#include "BlacksheepQuadX.hpp"
#include "PX4QuadX.hpp"

namespace msr { namespace airlib {

class PX4ConfigCreator {
public:
    static std::unique_ptr<MultiRotorParams> createConfig(const std::string& vehicle_name)
    {
        MavLinkDroneController::ConnectionInfo connection_info = getConnectionInfo(vehicle_name);

        std::unique_ptr<MultiRotorParams> config;

        if (connection_info.model == "Blacksheep") {
            config.reset(new BlacksheepQuadX(connection_info));
        }
        else {
            config.reset((new Px4QuadX(connection_info)));
        }

        return config;
    }

private:
    static MavLinkDroneController::ConnectionInfo getConnectionInfo(const std::string& connection_name)
    {
        //start with defaults
        MavLinkDroneController::ConnectionInfo connection_info;
        connection_info.vehicle_name = connection_name;

        //read settings and override defaults
        Settings& settings = Settings::singleton();
        if (settings.isLoadSuccess()) {
            Settings child;
            settings.getChild(connection_info.vehicle_name, child);

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

            connection_info.sitl_ip_address = child.getString("SitlIp", connection_info.sitl_ip_address);
            connection_info.sitl_ip_port = child.getInt("SitlPort", connection_info.sitl_ip_port);

            connection_info.local_host_ip = child.getString("LocalHostIp", connection_info.local_host_ip);


            connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);
            connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
            connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
            connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
            connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);
            connection_info.model = child.getString("Model", connection_info.model);
        }
        //do not write to users settings unless file didn't exist!
        else if (settings.hasFileName() && connection_info.vehicle_name.size() > 0) {
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

            child.setString("SitlIp", connection_info.sitl_ip_address);
            child.setInt("SitlPort", connection_info.sitl_ip_port);

            child.setString("LocalHostIp", connection_info.local_host_ip);

            child.setBool("UseSerial", connection_info.use_serial);
            child.setString("UdpIp", connection_info.ip_address);
            child.setInt("UdpPort", connection_info.ip_port);
            child.setString("SerialPort", connection_info.serial_port);
            child.setInt("SerialBaudRate", connection_info.baud_rate);
            child.setString("Model", connection_info.model);
            
            settings.setChild(connection_info.vehicle_name, child);
            settings.saveJSonFile("settings.json");
        }

        return connection_info;
    }

private:
    vector<unique_ptr<SensorBase>> sensor_storage_;
    string connection_name_;
};

}} //namespace
#endif

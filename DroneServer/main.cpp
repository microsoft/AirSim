// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <string>
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/multirotor/controllers/MavLinkDroneController.hpp"
#include "vehicles/multirotor/controllers/RealMultirotorConnector.hpp"
#include "common/Settings.hpp"

using namespace std;
using namespace msr::airlib;

void printUsage() {
    cout << "Usage: DroneServer" << endl;
    cout << "Start the DroneServer using the 'PX4' settings in ~/Documents/AirSim/settings.json." << endl;
}

int main(int argc, const char* argv[])
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " is_simulation" << std::endl;
        std::cout << "\t where is_simulation = 0 or 1" << std::endl;
        return 1;
    }

    bool is_simulation = std::atoi(argv[1]) == 1;
    if (is_simulation)
        std::cout << "You are running in simulation mode." << std::endl;
    else
        std::cout << "WARNING: This is not simulation!" << std::endl;

    MavLinkDroneController::ConnectionInfo connection_info;
    
    // read settings and override defaults
    Settings& settings = Settings::singleton().loadJSonFile("settings.json");
    Settings child;
    if (settings.isLoadSuccess()) {
        settings.getChild("PX4", child);

        // allow json overrides on a per-vehicle basis.
        connection_info.sim_sysid = static_cast<uint8_t>(child.getInt("SimSysID", connection_info.sim_sysid));
        connection_info.sim_compid = child.getInt("SimCompID", connection_info.sim_compid);

        connection_info.vehicle_sysid = static_cast<uint8_t>(child.getInt("VehicleSysID", connection_info.vehicle_sysid));
        connection_info.vehicle_compid = child.getInt("VehicleCompID", connection_info.vehicle_compid);

        connection_info.offboard_sysid = static_cast<uint8_t>(child.getInt("OffboardSysID", connection_info.offboard_sysid));
        connection_info.offboard_compid = child.getInt("OffboardCompID", connection_info.offboard_compid);

        connection_info.logviewer_ip_address = child.getString("LogViewerHostIp", connection_info.logviewer_ip_address);
        connection_info.logviewer_ip_port = child.getInt("LogViewerPort", connection_info.logviewer_ip_port);
        connection_info.logviewer_ip_sport = child.getInt("LogViewerSendPort", connection_info.logviewer_ip_sport);

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

    }
    else {
        std::cout << "Could not load settings from " << Settings::singleton().getFileName() << std::endl;
        return 3;

    }

    MavLinkDroneController mav_drone;
    mav_drone.initialize(connection_info, nullptr, is_simulation);
    mav_drone.reset();

    RealMultirotorConnector connector(& mav_drone);

    MultirotorApi server_wrapper(& connector);
    msr::airlib::MultirotorRpcLibServer server(&server_wrapper, connection_info.local_host_ip);
    
    //start server in async mode
    server.start(false);

    std::cout << "Server connected to MavLink endpoint at " << connection_info.local_host_ip << ":" << connection_info.ip_port << std::endl;
    std::cout << "Hit Ctrl+C to terminate." << std::endl;

    std::vector<std::string> messages;
    while (true) {
        //check messages
        server_wrapper.getStatusMessages(messages);
        if (messages.size() > 1) {
            for (const auto& message : messages) {
                std::cout << message << std::endl;
            }
        }        

        constexpr static std::chrono::milliseconds MessageCheckDurationMillis(100);
        std::this_thread::sleep_for(MessageCheckDurationMillis);

        mav_drone.reportTelemetry(100);
    }

    return 0;
}

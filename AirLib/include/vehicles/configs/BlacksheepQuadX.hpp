// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_BlacksheepQuadX_hpp
#define msr_airlib_vehicles_BlacksheepQuadX_hpp

#include "vehicles/MultiRotorParams.hpp"
#include "controllers/MavLinkDroneController.hpp"
#include "controllers/Settings.hpp"

//sensors
#include "sensors/barometer/BarometerSimple.hpp"
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"

namespace msr { namespace airlib {

class BlacksheepQuadX : public MultiRotorParams {
public:
	BlacksheepQuadX(const string& connection_name = string("Blacksheep"))
        : connection_name_(connection_name)
    {
    }

protected:
    virtual void setup(Params& params, SensorCollection& sensors, unique_ptr<DroneControllerBase>& controller) override
    {
		/*
		Motor placement:

		(2)  |   (0)
             |
		--------------
		     |
		(1)  |   (3)
		     |

		*/
        //set up arm lengths
        //dimensions are for Team Blacksheep Discovery (http://team-blacksheep.com/products/product:98)
        params.rotor_count = 4;
		std::vector<real_T> arm_lengths;
		// relative to Forward vector in the order (0,3,1,2) required by quad X pattern
		// http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
		arm_lengths.push_back(22);
		arm_lengths.push_back(25.5);
		arm_lengths.push_back(22);
		arm_lengths.push_back(25.5);

		std::vector<real_T> arm_angles; // note, this frame is not a perfect square X pattern.
		arm_angles.push_back(58);
		arm_angles.push_back(238);
		arm_angles.push_back(302);
		arm_angles.push_back(122);

		// quad X pattern 
		std::vector<RotorTurningDirection> rotor_directions;
		rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
		rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCCW);
		rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);
		rotor_directions.push_back(RotorTurningDirection::RotorTurningDirectionCW);


		// data from
		// http://dronesvision.net/team-blacksheep-750kv-motor-esc-set-for-tbs-discovery-fpv-quadcopter/
        //set up mass
        params.mass = 1.4f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.052f;  // weight for TBS motors 
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x = 0.20f; params.body_box.y = 0.12f; params.body_box.z = 0.04f;
        real_T rotor_z = 2.5f / 100;

        //computer rotor poses
		initializeRotors(params.rotor_poses, params.rotor_count, arm_lengths.data(), arm_angles.data(), rotor_directions.data(), rotor_z);
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
		Settings child;
        auto settings_filename = Settings::singleton().getFileName();
        if (!settings_filename.empty()) {
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
        }

        // update settings file with any new values that we now have.
        if (connection_info.vehicle_name.size() > 0) {
			
            bool changed = child.setInt("SimSysID", connection_info.sim_sysid);
            changed |= child.setInt("SimCompID", connection_info.sim_compid);

			changed |= child.setInt("VehicleSysID", connection_info.vehicle_sysid);
			changed |= child.setInt("VehicleCompID", connection_info.vehicle_compid);

			changed |= child.setInt("OffboardSysID", connection_info.offboard_sysid);
			changed |= child.setInt("OffboardCompID", connection_info.offboard_compid);

			changed |= child.setString("LogViewerHostIp", connection_info.logviewer_ip_address);
			changed |= child.setInt("LogViewerPort", connection_info.logviewer_ip_port);

			changed |= child.setString("QgcHostIp", connection_info.qgc_ip_address);
			changed |= child.setInt("QgcPort", connection_info.qgc_ip_port);

			changed |= child.setString("SitlIp", connection_info.sitl_ip_address);
			changed |= child.setInt("SitlPort", connection_info.sitl_ip_port);

			changed |= child.setString("LocalHostIp", connection_info.local_host_ip);

			changed |= child.setBool("UseSerial", connection_info.use_serial);
			changed |= child.setString("UdpIp", connection_info.ip_address);
			changed |= child.setInt("UdpPort", connection_info.ip_port);
			changed |= changed |= child.setString("SerialPort", connection_info.serial_port);
			changed |= child.setInt("SerialBaudRate", connection_info.baud_rate);

			if (changed) {
				settings.setChild(connection_info.vehicle_name, child);
				settings.saveJSonFile("settings.json");
			}
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

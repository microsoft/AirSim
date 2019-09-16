// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "common/Waiter.hpp"
#include "safety/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/lidar/LidarBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/distance/DistanceBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
Vehicle controller allows to obtain state from vehicle and send control commands to the vehicle.
State can include many things including sensor data, logs, estimated state from onboard computer etc.
Control commands can be low level actuation commands or high level movement commands.
The base class defines usually available methods that all vehicle controllers may implement.
Some methods may not be applicable to specific vehicle in which case an exception may be raised or call may be ignored.
*/
class VehicleApiBase : public UpdatableObject {
public:
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() const = 0;
    virtual bool armDisarm(bool arm) = 0;
    virtual GeoPoint getHomeGeoPoint() const = 0;

    virtual void update() override
    {
        UpdatableObject::update();
    }

    virtual void cancelLastTask()
    {
        //if derived class supports async task then override this method
    }
    virtual bool isReady(std::string& message) const
    {
        unused(message);
        return true;
    }

    //if vehicle supports it, call this method to send
    //kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
    virtual void sendTelemetry(float last_interval = -1)
    {
        //no default action
        unused(last_interval);
    }

    //below APIs are used by FastPhysicsEngine
    virtual real_T getActuation(unsigned int actuator_index) const
    {
        unused(actuator_index);
        throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
    }
    virtual size_t getActuatorCount() const
    {
        throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
    }

    virtual void getStatusMessages(std::vector<std::string>& messages)
    {
        unused(messages);
        //default implementation
    }

    /*
    For RCs, there are two cases: (1) vehicle may be configured to use
    RC bound to its hardware (2) vehicle may be configured to get RC data
    supplied via API calls. Below two APIs are not symmetrical, i.e.,
    getRCData() may or may not return same thing as setRCData().
    */
    //get reading from RC bound to vehicle (if unsupported then RCData::is_valid = false)
    virtual RCData getRCData() const
    {
        static const RCData invalid_rc_data {};
        return invalid_rc_data;
    }
    //set external RC data to vehicle (if unsupported then returns false)
    virtual bool setRCData(const RCData& rc_data)
    {
        unused(rc_data);
        return false;
    }

    // Sensors APIs
    virtual const SensorCollection& getSensors() const
    {
        throw VehicleCommandNotImplementedException("getSensors API is not supported for this vehicle");
    }

    // Lidar APIs
    virtual LidarData getLidarData(const std::string& lidar_name) const
    {
        auto *lidar = findLidarByName(lidar_name);
        if (lidar == nullptr)
            throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));

        return lidar->getOutput();
    }

    virtual vector<int> getLidarSegmentation(const std::string& lidar_name) const
    {
        auto *lidar = findLidarByName(lidar_name);
        if (lidar == nullptr)
            throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));

        return lidar->getSegmentationOutput();
    }

    // IMU API
    virtual ImuBase::Output getImuData(const std::string& imu_name) const
    {
        const ImuBase* imu = nullptr;

        // Find imu with the given name (for empty input name, return the first one found)
        // Not efficient but should suffice given small number of imus
        uint count_imus = getSensors().size(SensorBase::SensorType::Imu);
        for (uint i = 0; i < count_imus; i++)
        {
            const ImuBase* current_imu = static_cast<const ImuBase*>(getSensors().getByType(SensorBase::SensorType::Imu, i));
            if (current_imu != nullptr && (current_imu->getName() == imu_name || imu_name == ""))
            {
                imu = current_imu;
                break;
            }
        }
        if (imu == nullptr)
            throw VehicleControllerException(Utils::stringf("No IMU with name %s exist on vehicle", imu_name.c_str()));

        return imu->getOutput();
    }

    // Barometer API
    virtual BarometerBase::Output getBarometerData(const std::string& barometer_name) const
    {
        const BarometerBase* barometer = nullptr;

        uint count_barometers = getSensors().size(SensorBase::SensorType::Barometer);
        for (uint i = 0; i < count_barometers; i++)
        {
            const BarometerBase* current_barometer = static_cast<const BarometerBase*>(getSensors().getByType(SensorBase::SensorType::Barometer, i));
            if (current_barometer != nullptr && (current_barometer->getName() == barometer_name || barometer_name == ""))
            {
                barometer = current_barometer;
                break;
            }
        }
        if (barometer == nullptr)
            throw VehicleControllerException(Utils::stringf("No barometer with name %s exist on vehicle", barometer_name.c_str()));

        return barometer->getOutput();
    }

    // Magnetometer API
    virtual MagnetometerBase::Output getMagnetometerData(const std::string& magnetometer_name) const
    {
        const MagnetometerBase* magnetometer = nullptr;

        uint count_magnetometers = getSensors().size(SensorBase::SensorType::Magnetometer);
        for (uint i = 0; i < count_magnetometers; i++)
        {
            const MagnetometerBase* current_magnetometer = static_cast<const MagnetometerBase*>(getSensors().getByType(SensorBase::SensorType::Magnetometer, i));
            if (current_magnetometer != nullptr && (current_magnetometer->getName() == magnetometer_name || magnetometer_name == ""))
            {
                magnetometer = current_magnetometer;
                break;
            }
        }
        if (magnetometer == nullptr)
            throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));

        return magnetometer->getOutput();
    }

    // Gps API
    virtual GpsBase::Output getGpsData(const std::string& gps_name) const
    {
        const GpsBase* gps = nullptr;

        uint count_gps = getSensors().size(SensorBase::SensorType::Gps);
        for (uint i = 0; i < count_gps; i++)
        {
            const GpsBase* current_gps = static_cast<const GpsBase*>(getSensors().getByType(SensorBase::SensorType::Gps, i));
            if (current_gps != nullptr && (current_gps->getName() == gps_name || gps_name == ""))
            {
                gps = current_gps;
                break;
            }
        }
        if (gps == nullptr)
            throw VehicleControllerException(Utils::stringf("No gps with name %s exist on vehicle", gps_name.c_str()));

        return gps->getOutput();
    }

    // Distance Sensor API
    virtual DistanceBase::Output getDistanceSensorData(const std::string& distance_sensor_name) const
    {
        const DistanceBase* distance_sensor = nullptr;

        uint count_distance_sensors = getSensors().size(SensorBase::SensorType::Distance);
        for (uint i = 0; i < count_distance_sensors; i++)
        {
            const DistanceBase* current_distance_sensor = static_cast<const DistanceBase*>(getSensors().getByType(SensorBase::SensorType::Distance, i));
            if (current_distance_sensor != nullptr && (current_distance_sensor->getName() == distance_sensor_name || distance_sensor_name == ""))
            {
                distance_sensor = current_distance_sensor;
                break;
            }
        }
        if (distance_sensor == nullptr)
            throw VehicleControllerException(Utils::stringf("No distance sensor with name %s exist on vehicle", distance_sensor_name.c_str()));

        return distance_sensor->getOutput();
    }

    virtual ~VehicleApiBase() = default;

    //exceptions
    class VehicleControllerException : public std::runtime_error {
    public:
        VehicleControllerException(const std::string& message)
            : runtime_error(message) {
        }
    };

    class VehicleCommandNotImplementedException : public VehicleControllerException {
    public:
        VehicleCommandNotImplementedException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };

    class VehicleMoveException : public VehicleControllerException {
    public:
        VehicleMoveException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };

    private:
    const LidarBase* findLidarByName(const std::string& lidar_name) const
    {
        const LidarBase* lidar = nullptr;

        // Find lidar with the given name (for empty input name, return the first one found)
        // Not efficient but should suffice given small number of lidars
        uint count_lidars = getSensors().size(SensorBase::SensorType::Lidar);
        for (uint i = 0; i < count_lidars; i++)
        {
            const LidarBase* current_lidar = static_cast<const LidarBase*>(getSensors().getByType(SensorBase::SensorType::Lidar, i));
            if (current_lidar != nullptr && (current_lidar->getName() == lidar_name || lidar_name == ""))
            {
                lidar = current_lidar;
                break;
            }
        }

        return lidar;
    }
};


}} //namespace
#endif

#ifndef msr_airlib_SensorFactoryBase_hpp
#define msr_airlib_SensorFactoryBase_hpp

#include "SensorBase.hpp"
#include "SensorCollection.hpp"
#include <memory>

//sensors
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/barometer/BarometerSimple.hpp"

namespace msr
{
namespace airlib
{

    class SensorFactory
    {
    public:
        // creates one sensor from settings
        virtual std::shared_ptr<SensorBase> createSensorFromSettings(
            const AirSimSettings::SensorSetting* sensor_setting, const std::string& vehicle_type = "") const
        {
            unused(vehicle_type);

            switch (sensor_setting->sensor_type) {
            case SensorBase::SensorType::Imu:
                return std::shared_ptr<ImuSimple>(new ImuSimple(*static_cast<const AirSimSettings::ImuSetting*>(sensor_setting)));
            case SensorBase::SensorType::Magnetometer:
                return std::shared_ptr<MagnetometerSimple>(new MagnetometerSimple(*static_cast<const AirSimSettings::MagnetometerSetting*>(sensor_setting)));
            case SensorBase::SensorType::Gps:
                return std::shared_ptr<GpsSimple>(new GpsSimple(*static_cast<const AirSimSettings::GpsSetting*>(sensor_setting)));
            case SensorBase::SensorType::Barometer:
                return std::shared_ptr<BarometerSimple>(new BarometerSimple(*static_cast<const AirSimSettings::BarometerSetting*>(sensor_setting)));
            default:
                throw new std::invalid_argument("Unexpected sensor type");
            }
        }

        // creates sensor-collection
        virtual void createSensorsFromSettings(
            const std::map<std::string, std::shared_ptr<AirSimSettings::SensorSetting>>& sensors_settings,
            SensorCollection& sensors,
            vector<shared_ptr<SensorBase>>& sensor_storage,
            const std::string& vehicle_type = "") const
        {
            for (const auto& sensor_setting_pair : sensors_settings) {
                const AirSimSettings::SensorSetting* sensor_setting = sensor_setting_pair.second.get();

                // ignore sensors that are marked "disabled" in settings
                if (sensor_setting == nullptr || !sensor_setting->enabled)
                    continue;

                std::shared_ptr<SensorBase> sensor = createSensorFromSettings(sensor_setting, vehicle_type);
                if (sensor) {
                    sensor_storage.push_back(sensor);
                    sensors.insert(sensor.get(), sensor_setting->sensor_type);
                }
            }
        }

        virtual ~SensorFactory() = default;
    };
}
} //namespace
#endif
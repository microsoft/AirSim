#include "UnitySensorFactory.h"
#include "UnityDistanceSensor.h"

UnitySensorFactory::UnitySensorFactory(std::string vehicle_name, const NedTransform* ned_transform)
{
    setActor(vehicle_name, ned_transform);
}

void UnitySensorFactory::setActor(std::string vehicle_name, const NedTransform* ned_transform)
{
    vehicle_name_ = vehicle_name;
    ned_transform_ = ned_transform;
}

std::shared_ptr<msr::airlib::SensorBase> UnitySensorFactory::createSensorFromSettings(const AirSimSettings::SensorSetting* sensor_setting) const
{

    using SensorBase = msr::airlib::SensorBase;

    switch (sensor_setting->sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::make_shared<UnityDistanceSensor>(vehicle_name_, ned_transform_);
    default:
        return msr::airlib::SensorFactory::createSensorFromSettings(sensor_setting);
    }

    return std::shared_ptr<msr::airlib::SensorBase>();
}

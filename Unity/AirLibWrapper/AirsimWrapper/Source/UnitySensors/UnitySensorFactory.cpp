#include "UnitySensorFactory.h"
#include "UnityDistanceSensor.h"

UnitySensorFactory::UnitySensorFactory(std::string vehicle_name, const NedTransform* ned_transform)
{
	setActor(vehicle_name, ned_transform);
}

std::unique_ptr<msr::airlib::SensorBase> UnitySensorFactory::createSensor(msr::airlib::SensorBase::SensorType sensor_type) const
{
	using SensorBase = msr::airlib::SensorBase;

	switch (sensor_type)
	{
	case SensorBase::SensorType::Distance:
		return std::unique_ptr<UnityDistanceSensor>(new UnityDistanceSensor(vehicle_name_, ned_transform_));
	default:
		return msr::airlib::SensorFactory::createSensor(sensor_type);
	}
}

void UnitySensorFactory::setActor(std::string vehicle_name, const NedTransform* ned_transform)
{
	vehicle_name_ = vehicle_name;
	ned_transform_ = ned_transform;
}
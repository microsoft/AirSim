#pragma once

#include "sensors/SensorFactory.hpp"
#include "../NedTransform.h"

using namespace msr::airlib;

class UnitySensorFactory : public SensorFactory
{
public:
	UnitySensorFactory(std::string vehicle_name, const NedTransform* ned_transform);
	void setActor(std::string vehicle_name, const NedTransform* ned_transform);
	virtual std::unique_ptr<msr::airlib::SensorBase> createSensorFromSettings(const AirSimSettings::SensorSetting* sensor_setting) const override;

private:
	std::string vehicle_name_;
	const NedTransform* ned_transform_;
};

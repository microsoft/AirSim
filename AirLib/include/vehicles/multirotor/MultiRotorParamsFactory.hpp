// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_MultiRotorParamsFactory_hpp
#define msr_airlib_vehicles_MultiRotorParamsFactory_hpp

#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"


namespace msr { namespace airlib {

class MultiRotorParamsFactory {
public:

	static void reset();

	static std::unique_ptr<MultiRotorParams> createConfig(const AirSimSettings::VehicleSetting* vehicle_setting,
		std::shared_ptr<const SensorFactory> sensor_factory);

private:

	// Simple zero-based ID for ArduCopterSolo vehicles
	static int next_solo_id_;

};

}} //namespace
#endif

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY

#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "vehicles/multirotor/firmwares/mavlink/ArduCopterSoloParams.hpp"
#include "vehicles/multirotor/firmwares/mavlink/Px4MultiRotorParams.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightQuadXParams.hpp"

namespace msr { namespace airlib {

		int MultiRotorParamsFactory::next_solo_id_ = 0;

		void MultiRotorParamsFactory::reset()
		{
			next_solo_id_ = 0;
		}

		std::unique_ptr<MultiRotorParams> MultiRotorParamsFactory::createConfig(const AirSimSettings::VehicleSetting* vehicle_setting,
			std::shared_ptr<const SensorFactory> sensor_factory)
		{
			std::unique_ptr<MultiRotorParams> config;

			if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypePX4) {
				config.reset(new Px4MultiRotorParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting),
					sensor_factory));
			}
			else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo) {
				config.reset(new ArduCopterSoloParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), sensor_factory, next_solo_id_));
				next_solo_id_++;
			}
			else if (vehicle_setting->vehicle_type == "" || //default config
				vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) {
				config.reset(new SimpleFlightQuadXParams(vehicle_setting, sensor_factory));
			}
			else
				throw std::runtime_error(Utils::stringf(
					"Cannot create vehicle config because vehicle name '%s' is not recognized",
					vehicle_setting->vehicle_name.c_str()));

			config->initialize();

			return config;
		}
	}
}

#endif  /* AIRLIB_HEADER_ONLY */

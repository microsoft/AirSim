// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_ArduCopterSolo_hpp
#define msr_airlib_vehicles_ArduCopterSolo_hpp

#include "vehicles/multirotor/firmwares/mavlink/ArduCopterSoloApi.hpp"

namespace msr
{
namespace airlib
{

    class ArduCopterSoloParams : public MultiRotorParams
    {

    public:
        ArduCopterSoloParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_settings, std::shared_ptr<const SensorFactory> sensor_factory)
            : sensor_factory_(sensor_factory)
        {
            connection_info_ = getConnectionInfo(vehicle_settings);
        }

        virtual ~ArduCopterSoloParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            unique_ptr<MultirotorApiBase> api(new ArduCopterSoloApi());
            auto api_ptr = static_cast<ArduCopterSoloApi*>(api.get());
            api_ptr->initialize(connection_info_, &getSensors(), true);

            return api;
        }

        virtual void setupParams() override
        {
            // TODO consider whether we want to make us of the 'connection_info_.model' field (perhaps to indicate different sensor configs, say?  not sure...)

            auto& params = getParams();

            setupSolo(params);
        }

    private:
        void setupSolo(Params& params)
        {
            //set up arm lengths
            //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
            params.rotor_count = 4;

            // This is in meters (3DR spec indicates that motor-to-motor is 18.1 inches, so we're just dividing that by 2, which looks about right in real life)
            std::vector<real_T> arm_lengths(params.rotor_count, 0.22987f);

            // Takeoff mass, in kilograms (this is from the 3DR spec with no camera)
            //params.mass = 1.5f;
            // And this is what works currently - TODO sort out why ArduPilot isn't providing sufficient rotor power for default Solo weight - presumably some param setting?
            params.mass = 0.8f;

            // Mass in kg - can't find a 3DR spec on this, so we'll guess they're the same 55g as the DJI F450's motors
            real_T motor_assembly_weight = 0.055f;
            real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

            // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
            // given new thrust coefficients, motor max_rpm and propeller diameter.
            params.rotor_params.calculateMaxThrust();

            // Dimensions of core body box or abdomen, in meters (not including arms).  KM measures the Solo at 9.5" x 4.5" x 3"
            params.body_box.x() = 0.2413f;
            params.body_box.y() = 0.1143f;
            params.body_box.z() = 0.0762f;

            // Meters up from center of box mass - KM measures at about 3"
            real_T rotor_z = 0.0762f;

            //computer rotor poses
            initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

            //compute inertia matrix
            computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        }

        static const AirSimSettings::MavLinkConnectionInfo getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
        {
            AirSimSettings::MavLinkConnectionInfo result = vehicle_setting.connection_info;
            return result;
        }

    protected:
        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

    private:
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif

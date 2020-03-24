// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_ArduCopter_hpp
#define msr_airlib_vehicles_ArduCopter_hpp

#include "vehicles/multirotor/firmwares/arducopter/ArduCopterApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"


namespace msr { namespace airlib {

class ArduCopterParams : public MultiRotorParams {
public:
    ArduCopterParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
        : sensor_factory_(sensor_factory)
    {
        connection_info_ = getConnectionInfo(vehicle_setting);
    }

    virtual ~ArduCopterParams() = default;

    virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
    {
        return std::unique_ptr<MultirotorApiBase>(new ArduCopterApi(this, connection_info_));
    }

protected:
    virtual void setupParams() override
    {
        auto& params = getParams();

        // Use connection_info_.model for the model name, see Px4MultiRotorParams for example
        
        // Only Generic for now
        setupFrameGenericQuad(params);
    }

    virtual const SensorFactory* getSensorFactory() const override
    {
        return sensor_factory_.get();
    }

    static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
    {
        return vehicle_setting.connection_info;
    }

private:
    void setupFrameGenericQuad(Params& params)
    {
        /******* Below is same config as PX4 generic model ********/

        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

        //set up mass
        params.mass = 1.0f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        // using rotor_param default, but if you want to change any of the rotor_params, call calculateMaxThrust() to recompute the max_thrust
        // given new thrust coefficients, motor max_rpm and propeller diameter.
        params.rotor_params.calculateMaxThrust();

        //set up dimensions of core body box or abdomen (not including arms).
        params.body_box.x() = 0.180f; params.body_box.y() = 0.11f; params.body_box.z() = 0.040f;
        real_T rotor_z = 2.5f / 100;

        //computer rotor poses
        initializeRotorQuadX(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);

        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);

        //leave everything else to defaults
    }

    AirSimSettings::MavLinkConnectionInfo connection_info_;
    vector<unique_ptr<SensorBase>> sensor_storage_;
    // const AirSimSettings::MavlinkVehicleSetting* vehicle_setting_; //store as pointer because of derived classes
    std::shared_ptr<const SensorFactory> sensor_factory_;
};

}} //namespace
#endif

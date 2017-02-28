// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_Pix4QuadX_hpp
#define msr_airlib_vehicles_Pix4QuadX_hpp

#include "vehicles/MultiRotorParams.hpp"

//sensors
#include "sensors/barometer/BarometerSimple.hpp"
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"

namespace msr { namespace airlib {

class Px4QuadX : public MultiRotorParams {
protected:
    virtual void setup(Params& params, SensorCollection& sensors) override
    {
        //set up arm lengths
        //dimensions are for F450 frame: http://artofcircuits.com/product/quadcopter-frame-hj450-with-power-distribution
        params.rotor_count = 4;
        std::vector<real_T> arm_lengths(params.rotor_count, 0.2275f);

        //set up mass
        params.mass = 1.0f; //can be varied from 0.800 to 1.600
        real_T motor_assembly_weight = 0.055f;  //weight for MT2212 motor for F450 frame
        real_T box_mass = params.mass - params.rotor_count * motor_assembly_weight;

        //set up dimensions of core body box
        params.body_box.x = 0.180f; params.body_box.y = 0.11f; params.body_box.z = 0.040f;
        real_T rotor_z = 2.5f / 100;

        //computer rotor poses
        initializeRotorPoses(params.rotor_poses, params.rotor_count, arm_lengths.data(), rotor_z);
        //compute inertia matrix
        computeInertiaMatrix(params.inertia, params.body_box, params.rotor_poses, box_mass, motor_assembly_weight);
        //create sensors
        createStandardSensors(sensors, params.enabled_sensors);

        //leave everything else to defaults
    }

private:
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
};

}} //namespace
#endif

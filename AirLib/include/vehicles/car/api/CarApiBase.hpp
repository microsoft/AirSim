// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "common/ImageCaptureBase.hpp"
#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"

namespace msr { namespace airlib {

class CarApiBase : public VehicleApiBase  {
public:
    struct CarControls {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
            bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val),
            is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
        }
        void set_throttle(float throttle_val, bool forward)
        {
            if (forward) {
                is_manual_gear = false;
                manual_gear = 0;
                throttle = std::abs(throttle_val);
            }
            else {
                is_manual_gear = false;
                manual_gear = -1;
                throttle = - std::abs(throttle_val);
            }
        }
    };

    struct CarState {
        float speed;
        int gear;
        CollisionInfo collision;
        Kinematics::State kinematics_true;
        uint64_t timestamp;

        CarState(float speed_val, int gear_val, const CollisionInfo& collision_val, 
            const Kinematics::State& kinematics_true_val, uint64_t timestamp_val)
            : speed(speed_val), gear(gear_val), collision(collision_val), kinematics_true(kinematics_true_val), timestamp(timestamp_val)
        {
        }
    };

    virtual void setCarControls(const CarControls& controls) = 0;
    virtual CarState getCarState() = 0;
    virtual const CarApiBase::CarControls& getCarControls() const = 0;

    virtual ~CarApiBase() = default;
};


}} //namespace
#endif
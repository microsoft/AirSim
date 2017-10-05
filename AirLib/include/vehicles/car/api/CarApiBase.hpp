// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "controllers/VehicleCameraBase.hpp"
#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {

class CarApiBase {
public:
    struct CarControls {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = false;

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
        Vector3r position;
        Vector3r velocity;
        Quaternionr orientation;

        CarState(float speed_val, int gear_val, const Vector3r& position_val, const Vector3r& velocity_val,
            const Quaternionr& orientation_val)
            : speed(speed_val), gear(gear_val), position(position_val), velocity(velocity_val),
            orientation(orientation_val)
        {
        }
    };

    virtual vector<VehicleCameraBase::ImageResponse> simGetImages(const vector<VehicleCameraBase::ImageRequest>& request) = 0;
    virtual vector<uint8_t> simGetImage(uint8_t camera_id, VehicleCameraBase::ImageType image_type) = 0;
    virtual void setCarControls(const CarControls& controls) = 0;
    virtual CarState getCarState() = 0;
    virtual GeoPoint getHomeGeoPoint() = 0;
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() = 0;
    virtual void reset() = 0;
    virtual void simSetPose(const Pose& pose, bool ignore_collison) = 0;
    virtual Pose simGetPose() = 0;
    virtual ~CarApiBase() = default;
};


}} //namespace
#endif
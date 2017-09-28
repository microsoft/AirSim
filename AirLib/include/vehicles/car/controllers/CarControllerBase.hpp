// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarControllerBase_hpp
#define air_CarControllerBase_hpp

#include "controllers/VehicleCameraBase.hpp"
#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {

class CarControllerBase {
public:
    struct CarControls {
        float throttle = 0;
        float steering = 0;
        bool handbreak = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = false;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, bool handbreak_val,
            bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), handbreak(handbreak_val),
            is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
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

    virtual ~CarControllerBase() = default;
};


}} //namespace
#endif
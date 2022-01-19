#pragma once

#include "CommonStructs.hpp"

namespace simple_flight
{

class IStateEstimator
{
public:
    virtual Axis3r getAngles() const = 0;
    virtual Axis3r getAngularVelocity() const = 0;
    virtual Axis3r getPosition() const = 0;
    virtual Axis3r getLinearVelocity() const = 0;
    virtual Axis4r getOrientation() const = 0;
    virtual GeoPoint getGeoPoint() const = 0;

    virtual KinematicsState getKinematicsEstimated() const = 0;

    virtual GeoPoint getHomeGeoPoint() const = 0;

    virtual Axis3r transformToBodyFrame(const Axis3r& world_frame_val) const = 0;

<<<<<<< HEAD
    virtual simple_flight::SensorMeasurements getTrueMeasurements() const = 0;
    virtual simple_flight::SensorMeasurements getEkfMeasurements() const = 0;
    virtual simple_flight::Axis3r getEkfPosition() const = 0;
    virtual simple_flight::Axis3r getEkfLinearVelocity() const = 0;
    virtual simple_flight::Axis4r getEkfOrientation() const = 0;
    virtual simple_flight::Axis3r getEkfAngles() const = 0;
    virtual simple_flight::SensorBiases getEkfSensorBias() const = 0;
    virtual simple_flight::EkfKinematicsState getEkfKinematicsEstimated() const = 0;

    virtual simple_flight::Axis3r getEkfPositionCovariance() const = 0;
    virtual simple_flight::Axis3r getEkfLinearVelocityCovariance() const = 0;
    virtual simple_flight::Axis4r getEkfOrientationCovariance() const = 0;
    virtual simple_flight::Axis3r getEkfAnglesCovariance() const = 0;
    virtual simple_flight::Axis3r getEkfImuBiasCovariance() const = 0;
    virtual simple_flight::Axis3r getEkfGyroBiasCovariance() const = 0;
    virtual float getEkfBaroBiasCovariance() const = 0;
    
    virtual float getEkfOrientationNorm() const = 0;

    virtual std::array<float, 6> getEkfOrientationOffDiagCovariance() const = 0;
    virtual std::array<float, 12> getEkfOrientationGyroBiasCovariance() const = 0;

    virtual simple_flight::Axis3r getTrueAngles() const = 0;
    virtual simple_flight::Axis3r getTrueAngularVelocity() const = 0;
    virtual simple_flight::Axis3r getTruePosition() const = 0;
    virtual simple_flight::Axis3r getTrueLinearVelocity() const = 0;
    virtual simple_flight::Axis4r getTrueOrientation() const = 0;
    virtual simple_flight::KinematicsState getTrueKinematicsEstimated() const = 0;
=======
    virtual ~IStateEstimator() = default;
>>>>>>> 0becf9148d88c19c3ffd69359ebe1f9c3ae43826
};
}

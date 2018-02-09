#ifndef msr_airlib_SensorFactoryBase_hpp
#define msr_airlib_SensorFactoryBase_hpp


#include "SensorBase.hpp"
#include <memory>

//sensors
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"
#include "sensors/barometer/BarometerSimple.hpp"

namespace msr { namespace airlib {


class SensorFactory {
public:
    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) const
    {
        switch (sensor_type) {
        case SensorBase::SensorType::Imu:
            return std::unique_ptr<ImuSimple>(new ImuSimple());
        case SensorBase::SensorType::Magnetometer:
            return std::unique_ptr<MagnetometerSimple>(new MagnetometerSimple());
        case SensorBase::SensorType::Gps:
            return std::unique_ptr<GpsSimple>(new GpsSimple());
        case SensorBase::SensorType::Barometer:
            return std::unique_ptr<BarometerSimple>(new BarometerSimple());
        default:
            return std::unique_ptr<SensorBase>();
        }
    }
};


}} //namespace
#endif
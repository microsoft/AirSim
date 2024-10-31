#ifndef __IMAGE_METADATA__
#define __IMAGE_METADATA__

#include <chrono>
#include <string>
#include <vector>

#include <exiv2/exiv2.hpp>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/chrono.h>


namespace image_metadata {

class Metadata {
public:
    Metadata()
        : _rangefinder_distance(0.)
        , _drone_attitude()
        , _drone_gps()
        , _fix_state(0.)
        , _gimbal_attitude()
        , _gps_counter(0)
        , _horizontal_accuracy(0.)
        , _horizontal_dilution_precision(0.)
        , _mission_id()
        , _num_glonass_satellites_used(0)
        , _num_gps_satellites_used(0)
        , _num_total_satellites_used(0)
        , _position_dilution_precision(0.)
        , _relative_height(0.)
        , _rtk_connection_status(0)
        , _rtk_position()
        , _rtk_position_info(0)
        , _rtk_velocity()
        , _rtk_yaw(0)
        , _rtk_yaw_info(0)
        , _speed_accuracy(0.)
        , _time()
        , _vertical_accuracy(0.)
    {
    }

    Metadata(
        const double rangefinder_distance,
        const std::vector<float> drone_attitude,
        const std::vector<float> drone_gps,
        const float fix_state,
        const std::vector<float> gimbal_attitude,
        const short gps_counter,
        const float horizontal_accuracy,
        const float horizontal_dilution_precision,
        const std::string mission_id,
        const int num_glonass_satellites_used,
        const int num_gps_satellites_used,
        const int num_total_satellites_used,
        const float position_dilution_precision,
        const float relative_height,
        const int rtk_connection_status,
        const std::vector<double> rtk_position,
        const int rtk_position_info,
        const std::vector<float> rtk_velocity,
        const int rtk_yaw,
        const int rtk_yaw_info,
        const float speed_accuracy,
        const std::chrono::system_clock::time_point time,
        const float vertical_accuracy
    )
        : _rangefinder_distance(rangefinder_distance)
        , _drone_attitude(drone_attitude)
        , _drone_gps(drone_gps)
        , _fix_state(fix_state)
        , _gimbal_attitude(gimbal_attitude)
        , _gps_counter(gps_counter)
        , _horizontal_accuracy(horizontal_accuracy)
        , _horizontal_dilution_precision(horizontal_dilution_precision)
        , _mission_id(mission_id)
        , _num_glonass_satellites_used(num_glonass_satellites_used)
        , _num_gps_satellites_used(num_gps_satellites_used)
        , _num_total_satellites_used(num_total_satellites_used)
        , _position_dilution_precision(position_dilution_precision)
        , _relative_height(relative_height)
        , _rtk_connection_status(rtk_connection_status)
        , _rtk_position(rtk_position)
        , _rtk_position_info(rtk_position_info)
        , _rtk_velocity(rtk_velocity)
        , _rtk_yaw(rtk_yaw)
        , _rtk_yaw_info(rtk_yaw_info)
        , _speed_accuracy(speed_accuracy)
        , _time(time)
        , _vertical_accuracy(vertical_accuracy)
    {
    }

    const double& rangefinderDistance() const
    {
        return _rangefinder_distance;
    }

    const float& droneAttitudePitch() const
    {
        return _drone_attitude.at(1);
    }

    const float& droneAttitudeRoll() const
    {
        return _drone_attitude.at(0);
    }

    const float& droneAttitudeYaw() const
    {
        return _drone_attitude.at(2);
    }

    const std::string droneGpsAltitude() const
    {
        return double_to_str(static_cast<double>(_drone_gps.at(2)), 6);
    }

    const std::string droneGpsLatitude() const
    {
        return double_to_str(static_cast<double>(_drone_gps.at(0)), 6);
    }

    const std::string droneGpsLongitude() const
    {
        return double_to_str(static_cast<double>(_drone_gps.at(1)), 6);
    }

    const std::string fixState() const
    {
        return double_to_str(static_cast<double>(_fix_state), 6);
    }

    const std::vector<float>& gimbalAttitude() const
    {
        return _gimbal_attitude;
    }

    const float& gimbalAttitudePitch() const
    {
        return _gimbal_attitude.at(1);
    }

    const float& gimbalAttitudeRoll() const
    {
        return _gimbal_attitude.at(0);
    }

    const float& gimbalAttitudeYaw() const
    {
        return _gimbal_attitude.at(2);
    }

    const short& gpsCounter() const
    {
        return _gps_counter;
    }

    const std::string horizontalAccuracy() const
    {
        return double_to_str(static_cast<double>(_horizontal_accuracy), 6);
    }

    const std::string horizontalDilutionPrecision() const
    {
        return double_to_str(static_cast<double>(_horizontal_dilution_precision), 6);
    }

    const std::string& missionId() const
    {
        return _mission_id;
    }

    const int& numGlonassSatellitesUsed() const
    {
        return _num_glonass_satellites_used;
    }

    const int& numGpsSatellitesUsed() const
    {
        return _num_gps_satellites_used;
    }

    const int& numTotalSatellitesUsed() const
    {
        return _num_total_satellites_used;
    }

    const std::string positionDilutionPrecision() const
    {
        return double_to_str(static_cast<double>(_position_dilution_precision), 6);
    }

    const std::string relativeHeight() const
    {
        return double_to_str(static_cast<double>(_relative_height), 6);
    }

    const int& rtkConnectionStatus() const
    {
        return _rtk_connection_status;
    }

    const std::string rtkPositionAltitude() const
    {
        return double_to_str(_rtk_position.at(2), 8);
    }

    const std::string rtkPositionLatitude() const
    {
        return double_to_str(_rtk_position.at(0), 8);
    }

    const std::string rtkPositionLongitude() const
    {
        return double_to_str(_rtk_position.at(1), 8);
    }

    const int& rtkPositionInfo() const
    {
        return _rtk_position_info;
    }

    const std::string rtkVelocityX() const
    {
        return double_to_str(static_cast<double>(_rtk_velocity.at(0)), 6);
    }

    const std::string rtkVelocityY() const
    {
        return double_to_str(static_cast<double>(_rtk_velocity.at(1)), 6);
    }

    const std::string rtkVelocityZ() const
    {
        return double_to_str(static_cast<double>(_rtk_velocity.at(2)), 6);
    }

    const int& rtkYaw() const
    {
        return _rtk_yaw;
    }

    const int& rtkYawInfo() const
    {
        return _rtk_yaw_info;
    }

    const std::string speedAccuracy() const
    {
        return double_to_str(static_cast<double>(_speed_accuracy), 6);
    }

    const std::chrono::system_clock::time_point& time() const
    {
        return _time;
    }

    const std::string verticalAccuracy() const
    {
        return double_to_str(static_cast<double>(_vertical_accuracy), 6);
    }

private:
    const double _rangefinder_distance;

    const std::vector<float> _drone_attitude;

    const std::vector<float> _drone_gps;

    const float _fix_state;

    const std::vector<float> _gimbal_attitude;

    const short _gps_counter;

    const float _horizontal_accuracy;

    const float _horizontal_dilution_precision;

    const std::string _mission_id;

    const int _num_glonass_satellites_used;

    const int _num_gps_satellites_used;

    const int _num_total_satellites_used;

    const float _position_dilution_precision;

    const float _relative_height;

    const int _rtk_connection_status;

    const std::vector<double> _rtk_position;

    const int _rtk_position_info;

    const std::vector<float> _rtk_velocity;

    const int _rtk_yaw;

    const int _rtk_yaw_info;

    const float _speed_accuracy;

    const std::chrono::system_clock::time_point _time;

    const float _vertical_accuracy;

    const std::string double_to_str(double value, int precision) const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }
};

class Image {
public:
    Image(std::string file_path)
        : _image(Exiv2::ImageFactory::open(file_path))
    {
    }

    Image(const uint8_t* data, std::size_t size)
        : _image(Exiv2::ImageFactory::open(data, size))
    {
    }

    void output(std::vector<uint8_t>& data)
    {
        uint8_t* byte_data = _image->io().mmap();
        size_t size = _image->io().size();
        data.assign(byte_data, byte_data + size);
    }

    void update(const Metadata metadata)
    {
        _image->readMetadata();
        updateXmp(metadata);
        updateExif(metadata);

        // This could throw and should be caught by the consumer.
        _image->writeMetadata();
    }

private:
    Exiv2::Image::AutoPtr _image;

    void updateExif(const Metadata& metadata)
    {
        Exiv2::ExifData& exif_data = _image->exifData();

        exif_data["Exif.Photo.SubjectDistance"] =
            Exiv2::floatToRationalCast(metadata.rangefinderDistance());

        auto metadata_time_point_seconds =
            std::chrono::time_point_cast<std::chrono::seconds>(metadata.time());
        exif_data["Exif.Photo.DateTimeOriginal"] =
            fmt::format("{:%Y:%m:%d %H:%M:%S}", metadata_time_point_seconds);
        exif_data["Exif.Image.DateTime"] =
            fmt::format("{:%Y:%m:%d %H:%M:%S}", metadata_time_point_seconds);
    }

    void updateXmp(const Metadata& metadata)
    {
        Exiv2::XmpData& xmp_data = _image->xmpData();

        Exiv2::XmpProperties::registerNs("Zeitview/", "zv");

        xmp_data["Xmp.zv.AutoFlightMeta"] = metadata.missionId();

        xmp_data["Xmp.zv.RangefinderDistance"] = metadata.rangefinderDistance();

        xmp_data["Xmp.zv.AttitudeRoll"] = metadata.droneAttitudeRoll();
        xmp_data["Xmp.zv.AttitudePitch"] = metadata.droneAttitudePitch();
        xmp_data["Xmp.zv.AttitudeYaw"] = metadata.droneAttitudeYaw();

        xmp_data["Xmp.zv.GimbalAttitudeRoll"] = metadata.gimbalAttitudeRoll();
        xmp_data["Xmp.zv.GimbalAttitudePitch"] = metadata.gimbalAttitudePitch();
        xmp_data["Xmp.zv.GimbalAttitudeYaw"] = metadata.gimbalAttitudeYaw();

        xmp_data["Xmp.zv.RtkLatitude"] = metadata.rtkPositionLatitude();
        xmp_data["Xmp.zv.RtkLongitude"] = metadata.rtkPositionLongitude();
        xmp_data["Xmp.zv.RtkAltitude"] = metadata.rtkPositionAltitude();

        xmp_data["Xmp.zv.RtkVelocityX"] = metadata.rtkVelocityX();
        xmp_data["Xmp.zv.RtkVelocityY"] = metadata.rtkVelocityY();
        xmp_data["Xmp.zv.RtkVelocityZ"] = metadata.rtkVelocityZ();

        xmp_data["Xmp.zv.RtkYaw"] = metadata.rtkYaw();

        xmp_data["Xmp.zv.RtkYawInfo"] = metadata.rtkYawInfo();

        xmp_data["Xmp.zv.RtkPositionInfo"] = metadata.rtkPositionInfo();

        xmp_data["Xmp.zv.GpsLatitude"] = metadata.droneGpsLatitude();
        xmp_data["Xmp.zv.GpsLongitude"] = metadata.droneGpsLongitude();
        xmp_data["Xmp.zv.GpsAltitude"] = metadata.droneGpsAltitude();

        xmp_data["Xmp.zv.RelativeHeight"] = metadata.relativeHeight();

        xmp_data["Xmp.zv.RtkConnectionStatus"] = metadata.rtkConnectionStatus();
        
        xmp_data["Xmp.zv.HorizontalDilutionPrecision"] = metadata.horizontalDilutionPrecision();
        xmp_data["Xmp.zv.PositionDilutionPrecision"] = metadata.positionDilutionPrecision();
        xmp_data["Xmp.zv.FixState"] = metadata.fixState();
        xmp_data["Xmp.zv.VerticalAccuracy"] = metadata.verticalAccuracy();
        xmp_data["Xmp.zv.HorizontalAccuracy"] = metadata.horizontalAccuracy();
        xmp_data["Xmp.zv.SpeedAccuracy"] = metadata.speedAccuracy();
        xmp_data["Xmp.zv.NumGpsSatellitesUsed"] = metadata.numGpsSatellitesUsed();
        xmp_data["Xmp.zv.NumGlonassSatellitesUsed"] = metadata.numGlonassSatellitesUsed();
        xmp_data["Xmp.zv.NumTotalSatellitesUsed"] = metadata.numTotalSatellitesUsed();
        xmp_data["Xmp.zv.GpsCounter"] = metadata.gpsCounter();
    }
};

} // namespace image_metadata

#endif // __IMAGE_METADATA__

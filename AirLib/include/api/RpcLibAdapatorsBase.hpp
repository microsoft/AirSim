// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibAdapatorsBase_hpp
#define air_RpcLibAdapatorsBase_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"
#include "common/ImageCaptureBase.hpp"
#include "safety/SafetyEval.hpp"
#include "rpc/msgpack.hpp"


namespace msr { namespace airlib_rpclib {

class RpcLibAdapatorsBase {
public:
    template<typename TSrc, typename TDest>
    static void to(const std::vector<TSrc>& s, std::vector<TDest>& d)
    {
        d.clear();
        for (size_t i = 0; i < s.size(); ++i)
            d.push_back(s.at(i).to());
    }

    template<typename TSrc, typename TDest>
    static void from(const std::vector<TSrc>& s, std::vector<TDest>& d)
    {
        d.clear();
        for (size_t i = 0; i < s.size(); ++i)
            d.push_back(TDest(s.at(i)));
    }

    struct Vector3r {
        msr::airlib::real_T x_val = 0, y_val = 0, z_val = 0;
        MSGPACK_DEFINE_MAP(x_val, y_val, z_val);

        Vector3r()
        {}

        Vector3r(const msr::airlib::Vector3r& s)
        {
            x_val = s.x();
            y_val = s.y();
            z_val = s.z();
        }
        msr::airlib::Vector3r to() const
        {
            return msr::airlib::Vector3r(x_val, y_val, z_val);
        }
    };

    struct CollisionInfo {
        bool has_collided = false;
        Vector3r normal;
        Vector3r impact_point;
        Vector3r position;
        msr::airlib::real_T penetration_depth = 0;
        msr::airlib::TTimePoint time_stamp = 0;
        std::string object_name;
        int object_id = -1;

        MSGPACK_DEFINE_MAP(has_collided, penetration_depth, time_stamp, normal, impact_point, position, object_name, object_id);
        
        CollisionInfo()
        {}

        CollisionInfo(const msr::airlib::CollisionInfo& s)
        {
            has_collided = s.has_collided;
            normal = s.normal;
            impact_point = s.impact_point;
            position = s.position;
            penetration_depth = s.penetration_depth;
            time_stamp = s.time_stamp;
            object_name = s.object_name;
            object_id = s.object_id;
        }

        msr::airlib::CollisionInfo to() const
        {
            return msr::airlib::CollisionInfo(has_collided, normal.to(), impact_point.to(), position.to(),
                penetration_depth, time_stamp, object_name, object_id);
        }
    };

    struct Quaternionr {
        msr::airlib::real_T w_val = 1, x_val = 0, y_val = 0, z_val = 0;
        MSGPACK_DEFINE_MAP(w_val, x_val, y_val, z_val);

        Quaternionr()
        {}

        Quaternionr(const msr::airlib::Quaternionr& s)
        {
            w_val = s.w();
            x_val = s.x();
            y_val = s.y();
            z_val = s.z();
        }
        msr::airlib::Quaternionr to() const
        {
            return msr::airlib::Quaternionr(w_val, x_val, y_val, z_val);
        }
    };

    struct Pose {
        Vector3r position;
        Quaternionr orientation;
        MSGPACK_DEFINE_MAP(position, orientation);

        Pose()
        {}
        Pose(const msr::airlib::Pose& s)
        {
            position = s.position;
            orientation = s.orientation;
        }
        msr::airlib::Pose to() const
        {
            return msr::airlib::Pose(position.to(), orientation.to());
        }
    };

    struct GeoPoint {
        double latitude = 0, longitude = 0;
        float altitude = 0;
        MSGPACK_DEFINE_MAP(latitude, longitude, altitude);

        GeoPoint()
        {}

        GeoPoint(const msr::airlib::GeoPoint& s)
        {
            latitude = s.latitude;
            longitude = s.longitude;
            altitude = s.altitude;
        }
        msr::airlib::GeoPoint to() const
        {
            return msr::airlib::GeoPoint(latitude, longitude, altitude);
        }
    };

    struct RCData {
        uint64_t timestamp = 0;
        float pitch = 0, roll = 0, throttle = 0, yaw = 0;
        unsigned int  switch1 = 0, switch2 = 0, switch3 = 0, switch4 = 0, 
            switch5 = 0, switch6 = 0, switch7 = 0, switch8 = 0;
        bool is_initialized = false; //is RC connected?
        bool is_valid = false; //must be true for data to be valid

        MSGPACK_DEFINE_MAP(timestamp, pitch, roll, throttle, yaw, switch1, switch2, switch3, switch4, switch5, switch6, switch7, switch8, is_initialized, is_valid);

        RCData()
        {}

        RCData(const msr::airlib::RCData& s)
        {
            timestamp = s.timestamp;
            pitch = s.pitch;
            roll = s.roll;
            throttle = s.throttle;
            yaw = s.yaw;
            switch1 = s.switch1;
            switch2 = s.switch2;
            switch3 = s.switch3;
            switch4 = s.switch4;
            switch5 = s.switch5;
            switch6 = s.switch6;
            switch7 = s.switch7;
            switch8 = s.switch8;
            is_initialized = s.is_initialized;
            is_valid = s.is_valid;

        }
        msr::airlib::RCData to() const
        {
            msr::airlib::RCData d;
            d.timestamp = timestamp;
            d.pitch = pitch;
            d.roll = roll;
            d.throttle = throttle;
            d.yaw = yaw;
            d.switch1 = switch1;
            d.switch2 = switch2;
            d.switch3 = switch3;
            d.switch4 = switch4;
            d.switch5 = switch5;
            d.switch6 = switch6;
            d.switch7 = switch7;
            d.switch8 = switch8;
            d.is_initialized = is_initialized;
            d.is_valid = is_valid;
            
            return d;
        }
    };

    struct CameraInfo {
        Pose pose;
        float fov;

        MSGPACK_DEFINE_MAP(pose, fov);

        CameraInfo()
        {}

        CameraInfo(const msr::airlib::CameraInfo& s)
        {
            pose = s.pose;
            fov = s.fov;
        }

        msr::airlib::CameraInfo to() const
        {
            msr::airlib::CameraInfo s;
            s.pose = pose.to();
            s.fov = fov;

            return s;
        }
    };
    
    struct KinematicsState {
        Vector3r position;
        Quaternionr orientation;

        Vector3r linear_velocity;
        Vector3r angular_velocity;

        Vector3r linear_acceleration;
        Vector3r angular_acceleration;

        MSGPACK_DEFINE_MAP(position, orientation, linear_velocity, angular_velocity, linear_acceleration, angular_acceleration);


        KinematicsState()
        {}

        KinematicsState(const msr::airlib::Kinematics::State& s)
        {
            position = s.pose.position;
            orientation = s.pose.orientation;
            linear_velocity = s.twist.linear;
            angular_velocity = s.twist.angular;
            linear_acceleration = s.accelerations.linear;
            angular_acceleration = s.accelerations.angular;
        }

        msr::airlib::Kinematics::State to() const
        {
            msr::airlib::Kinematics::State s;
            s.pose.position = position.to();
            s.pose.orientation = orientation.to();
            s.twist.linear = linear_velocity.to();
            s.twist.angular = angular_velocity.to();
            s.accelerations.linear = linear_acceleration.to();
            s.accelerations.angular = angular_acceleration.to();

            return s;
        }
    };

    struct ImageRequest {
        uint8_t camera_id;
        msr::airlib::ImageCaptureBase::ImageType image_type;
        bool pixels_as_float;
        bool compress;

        MSGPACK_DEFINE_MAP(camera_id, image_type, pixels_as_float, compress);

        ImageRequest()
        {}

        ImageRequest(const msr::airlib::ImageCaptureBase::ImageRequest& s)
        {
            camera_id = s.camera_id;
            image_type = s.image_type;
            pixels_as_float = s.pixels_as_float;
            compress = s.compress;
        }

        msr::airlib::ImageCaptureBase::ImageRequest to() const
        {
            msr::airlib::ImageCaptureBase::ImageRequest d;
            d.camera_id = camera_id;
            d.image_type = image_type;
            d.pixels_as_float = pixels_as_float;
            d.compress = compress;

            return d;
        }

        static std::vector<ImageRequest> from(
            const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& request
        ) {
            std::vector<ImageRequest> request_adaptor;
            for (const auto& item : request)
                request_adaptor.push_back(ImageRequest(item));

            return request_adaptor;
        }
        static std::vector<msr::airlib::ImageCaptureBase::ImageRequest> to(
            const std::vector<ImageRequest>& request_adapter
        ) {
            std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request;
            for (const auto& item : request_adapter)
                request.push_back(item.to());

            return request;
        }         
    };

    struct ImageResponse {
        std::vector<uint8_t> image_data_uint8;
        std::vector<float> image_data_float;

        Vector3r camera_position;
        Quaternionr camera_orientation;
        msr::airlib::TTimePoint time_stamp;
        std::string message;
        bool pixels_as_float;
        bool compress;
        int width, height;
        msr::airlib::ImageCaptureBase::ImageType image_type;

        MSGPACK_DEFINE_MAP(image_data_uint8, image_data_float, camera_position, 
            camera_orientation, time_stamp, message, pixels_as_float, compress, width, height, image_type);

        ImageResponse()
        {}

        ImageResponse(const msr::airlib::ImageCaptureBase::ImageResponse& s)
        {
            pixels_as_float = s.pixels_as_float;
            
            image_data_uint8 = s.image_data_uint8;
            image_data_float = s.image_data_float;

            //TODO: remove bug workaround for https://github.com/rpclib/rpclib/issues/152
            if (image_data_uint8.size() == 0)
                image_data_uint8.push_back(0);
            if (image_data_float.size() == 0)
                image_data_float.push_back(0);

            camera_position = Vector3r(s.camera_position);
            camera_orientation = Quaternionr(s.camera_orientation);
            time_stamp = s.time_stamp;
            message = s.message;
            compress = s.compress;
            width = s.width;
            height = s.height;
            image_type = s.image_type;
        }

        msr::airlib::ImageCaptureBase::ImageResponse to() const
        {
            msr::airlib::ImageCaptureBase::ImageResponse d;

            d.pixels_as_float = pixels_as_float;

            if (! pixels_as_float)
                d.image_data_uint8 = image_data_uint8;
            else
                d.image_data_float = image_data_float;

            d.camera_position = camera_position.to();
            d.camera_orientation = camera_orientation.to();
            d.time_stamp = time_stamp;
            d.message = message;
            d.compress = compress;
            d.width = width;
            d.height = height;
            d.image_type = image_type;

            return d;
        }

        static std::vector<msr::airlib::ImageCaptureBase::ImageResponse> to(
            const std::vector<ImageResponse>& response_adapter
        ) {
            std::vector<msr::airlib::ImageCaptureBase::ImageResponse> response;
            for (const auto& item : response_adapter)
                response.push_back(item.to());

            return response;
        }
        static std::vector<ImageResponse> from(
            const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& response
        ) {
            std::vector<ImageResponse> response_adapter;
            for (const auto& item : response)
                response_adapter.push_back(ImageResponse(item));

            return response_adapter;
        }
    };
};

}} //namespace

MSGPACK_ADD_ENUM(msr::airlib::SafetyEval::SafetyViolationType_);
MSGPACK_ADD_ENUM(msr::airlib::SafetyEval::ObsAvoidanceStrategy);
MSGPACK_ADD_ENUM(msr::airlib::ImageCaptureBase::ImageType);


#endif

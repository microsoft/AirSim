#pragma once


#include "common/Common.hpp"
#include "rpc/RpcLibClient.hpp"
#include "controllers/DroneControllerBase.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

class RandomPointPoseGenerator {
public:
    RandomPointPoseGenerator(msr::airlib::RpcLibClient* client)
        : client_(client), 
        rand_xy_(-500.0f, 500.0f), rand_z_(-10.0f, -1.0f), rand_pitch_(-M_PIf / 8, M_PIf / 8),
        rand_yaw_(-M_PIf, M_PIf)
    {
    }

    void next()
    {
        const auto& collision_info = client_->getCollisionInfo();
        auto position = client_->getPosition();
        auto orientation = client_->getOrientation();

        if (collision_info.has_collided) {
            position = collision_info.position + collision_info.normal*2 + collision_info.normal * collision_info.penetration_depth * 2;
        }
        else {
            position.x() = rand_xy_.next();
            position.y() = rand_xy_.next();
            position.z() = rand_z_.next();

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(orientation, pitch, roll, yaw);
            pitch = rand_pitch_.next();
            yaw = rand_yaw_.next();

            orientation = VectorMath::toQuaternion(pitch, roll, yaw);
        }

        client_->simSetPosition(position);
        client_->simSetOrientation(orientation);
    }
private:
    typedef common_utils::RandomGeneratorF RandomGeneratorF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;


    msr::airlib::RpcLibClient* client_;
    RandomGeneratorF rand_xy_, rand_z_, rand_pitch_, rand_yaw_;
};

class RandomWalkPoseGenerator {
public:
    RandomWalkPoseGenerator(msr::airlib::RpcLibClient* client)
        : client_(client), 
        rand_xy_(-1.0f, 1.0f), rand_z_(-0.2f, 0.2f), rand_pitch_yaw_(-2 * M_PIf / 360, 2 * M_PIf / 360),
        min_position_(-1000, -1000, -10), max_position_(1000, 1000, 0), min_pitch_(-0.25f * M_PIf), max_pitch_(0.25f * M_PIf)
    {
    }

    void next()
    {
        const auto& collision_info = client_->getCollisionInfo();
        auto position = client_->getPosition();
        auto orientation = client_->getOrientation();

        if (collision_info.has_collided) {
            position = collision_info.position + collision_info.normal*2 + collision_info.normal * collision_info.penetration_depth * 2;
        }
        else {
            position.x() += rand_xy_.next();
            position.y() += rand_xy_.next();
            position.z() += rand_z_.next();

            position.x() = Utils::clip(position.x(), min_position_.x(), max_position_.x());
            position.y() = Utils::clip(position.y(), min_position_.y(), max_position_.y());
            position.z() = Utils::clip(position.z(), min_position_.z(), max_position_.z());

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(orientation, pitch, roll, yaw);
            pitch += rand_pitch_yaw_.next();
            yaw += rand_pitch_yaw_.next();

            pitch = Utils::clip(pitch, min_pitch_, max_pitch_);

            orientation = VectorMath::toQuaternion(pitch, roll, yaw);
        }

        client_->simSetPosition(position);
        client_->simSetOrientation(orientation);
    }
private:
    typedef common_utils::RandomGeneratorF RandomGeneratorF;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;

    msr::airlib::RpcLibClient* client_;
    RandomGeneratorF rand_xy_, rand_z_, rand_pitch_yaw_;

    Vector3r min_position_, max_position_;
    float min_pitch_, max_pitch_;
};
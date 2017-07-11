#pragma once


#include "common/Common.hpp"

class RandomPointPoseGenerator {
public:
private:
    typedef common_utils::RandomGeneratorGaussianF RandomGeneratorGaussianF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;

public:
    Vector3r position;
    Quaternionr orientation;

public:
    RandomPointPoseGenerator(int random_seed)
        :
        rand_xy_(0.0f, 150.0f), rand_z_(2.0f, 3.0f), rand_pitch_(0.0f, M_PIf / 2),
        rand_yaw_(0.0f, M_PIf)
    {
        rand_xy_.seed(random_seed);
        rand_z_.seed(random_seed);
        rand_pitch_.seed(random_seed);
        rand_yaw_.seed(random_seed);
    }

    void next()
    {
        position.x() = rand_xy_.next();
        position.y() = rand_xy_.next();
        position.z() = Utils::clip(rand_z_.next(), -10.0f, 0.0f);

        float pitch, roll, yaw;
        VectorMath::toEulerianAngle(orientation, pitch, roll, yaw);
        pitch = rand_pitch_.next();
        yaw = rand_yaw_.next();

        orientation = VectorMath::toQuaternion(pitch, 0, yaw);
    }
private:
    RandomGeneratorGaussianF rand_xy_, rand_z_, rand_pitch_, rand_yaw_;
};

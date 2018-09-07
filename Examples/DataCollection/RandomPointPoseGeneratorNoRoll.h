#pragma once


#include "common/Common.hpp"

class RandomPointPoseGeneratorNoRoll {
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
    RandomPointPoseGeneratorNoRoll(int random_seed)
        :
        //settings are for neighbourhood environement
        //sigma = desired_max / 2 so 95% of the times we in desired
        rand_xy_(0.0f, 75.0f), rand_z_(-5.0f, 4.0f), 
        rand_pitch_(0.0f, M_PIf / 8),
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
        position.z() = Utils::clip(rand_z_.next(), -10.0f, -1.0f);

        float pitch = Utils::clip(rand_pitch_.next(), -M_PIf / 2, M_PIf / 2);
        float yaw = Utils::clip(rand_yaw_.next(), -M_PIf, M_PIf);

        orientation = VectorMath::toQuaternion(pitch, 0, yaw);
    }
private:
    RandomGeneratorGaussianF rand_xy_, rand_z_, rand_pitch_, rand_yaw_;
};
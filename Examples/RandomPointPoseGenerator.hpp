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
        //settings are for neighbourhood environement
        //sigma = desired_max / 2 so 95% of the times we in desired
        rand_xy_(0.0f, 75.0f), rand_z_(2.0f, 1.0f), 
        rand_pitch_(0.0f, M_PIf / 8), rand_roll_(0.0f, M_PIf / 16),
        rand_yaw_(0.0f, M_PIf / 2)
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
        float roll = Utils::clip(rand_pitch_.next(), -M_PIf / 4, M_PIf / 4);
        float yaw = Utils::clip(rand_yaw_.next(), -M_PIf, M_PIf);

        orientation = VectorMath::toQuaternion(pitch, roll, yaw);
    }
private:
    RandomGeneratorGaussianF rand_xy_, rand_z_, rand_pitch_, rand_yaw_, rand_roll_;
};

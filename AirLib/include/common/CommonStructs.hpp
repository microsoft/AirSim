// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_CommonStructs_hpp
#define msr_air_copter_sim_CommonStructs_hpp

#include "common/Common.hpp"
#include <ostream>

namespace msr { namespace airlib {

//velocity
struct Twist {
    Vector3r linear, angular;

    Twist()
    {}

    Twist(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Twist zero()
    {
        static const Twist zero_twist(Vector3r::Zero(), Vector3r::Zero());
        return zero_twist;
    }
};

//force & torque
struct Wrench {
    Vector3r force, torque;

    Wrench()
    {}

    Wrench(const Vector3r& force_val, const Vector3r& torque_val)
        : force(force_val), torque(torque_val)
    {
    }

    //support basic arithmatic 
    Wrench operator+(const Wrench& other)
    {
        Wrench result;
        result.force = this->force + other.force;
        result.torque = this->torque + other.torque;
        return result;
    }
    Wrench operator+=(const Wrench& other)
    {
        force += other.force;
        torque += other.torque;
        return *this;
    }
    Wrench operator-(const Wrench& other)
    {
        Wrench result;
        result.force = this->force - other.force;
        result.torque = this->torque - other.torque;
        return result;
    }
    Wrench operator-=(const Wrench& other)
    {
        force -= other.force;
        torque -= other.torque;
        return *this;
    }

    static const Wrench zero()
    {
        static const Wrench zero_wrench(Vector3r::Zero(), Vector3r::Zero());
        return zero_wrench;
    }
};

struct Accelerations {
    Vector3r linear;
    Vector3r angular;

    Accelerations()
    {}

    Accelerations(const Vector3r& linear_val, const Vector3r& angular_val)
        : linear(linear_val), angular(angular_val)
    {
    }

    static const Accelerations zero()
    {
        static const Accelerations zero_accelerations(Vector3r::Zero(), Vector3r::Zero());
        return zero_accelerations;
    }
};

struct PoseWithCovariance {
	VectorMath::Pose pose;
	vector<real_T> covariance;	//36 elements, 6x6 matrix

    PoseWithCovariance()
        : covariance(36, 0)
    {}
};

struct PowerSupply {
	vector<real_T> voltage, current;
};

struct TwistWithCovariance {
	Twist twist;
	vector<real_T> covariance;	//36 elements, 6x6 matrix

    TwistWithCovariance()
        : covariance(36, 0)
    {}
};

struct Joystick {
	vector<float> axes;
	vector<int> buttons;
};

struct Odometry {
	PoseWithCovariance pose;
	TwistWithCovariance twist;
};

struct GeoPoint {
    double latitude = 0, longitude = 0;
    float altitude = 0;

    GeoPoint()
    {}

    GeoPoint(double latitude_val, double longitude_val, float altitude_val)
    {
        set(latitude_val, longitude_val, altitude_val);
    }

    void set(double latitude_val, double longitude_val, float altitude_val)
    {
        latitude = latitude_val, longitude = longitude_val; altitude = altitude_val;
    }

    friend std::ostream& operator<<(std::ostream &os, GeoPoint const &g) { 
        return os << "[" << g.latitude << ", " << g.longitude << ", " << g.altitude << "]";
    }

    std::string to_string()
    {
        return std::to_string(latitude) + string(", ") + std::to_string(longitude) + string(", ") + std::to_string(altitude);
    }
};

struct CollisionInfo {
    bool has_collided = false;
    int collison_count = 0;
    Vector3r normal = Vector3r::Zero();
    Vector3r impact_point = Vector3r::Zero();
    Vector3r position = Vector3r::Zero();
    real_T penetration_depth = 0;
};

struct GeoPose {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Quaternionr orientation;
	GeoPoint position;
};



}} //namespace
#endif

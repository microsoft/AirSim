// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_vehicles_firefly_hpp
#define msr_air_copter_sim_vehicles_firefly_hpp

#include "vehicles/MultiRotorParams.hpp"

namespace msr { namespace airlib {

class Firefly {
private:
    typedef msr::airlib::MultiRotorParams  MultiRotorParams;
    typedef MultiRotorParams::RotorPose RotorPose;
    MultiRotorParams params_;
public:
	Firefly()
	{
        params_.initializeRotorPoses(4);
		params_.mass = 1.0;
        params_.inertia = Matrix3x3r::Identity();
	}

    const MultiRotorParams& getParams()
    {
        return params_;
    }

	static const MultiRotorParams& Params()
	{
		static Firefly vehicle_;	
		return vehicle_.getParams();
	}

};

}} //namespace
#endif

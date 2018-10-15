#pragma once

#include "common/Common.hpp"
#include "AirSimStructs.hpp"

/*
Note on coordinate system
-------------------------
We have following coordinate systems:
(1) UU or Unreal Units or Unreal Coordinate system
(2) Global NED: This is NED transformation of UU with origin set to 0,0,0
(3) Local NED: This is NED transformation of UU with origin set to vehicle's spawning UU location
(4) Geo: This is GPS coordinate where UU origin is assigned some geo-coordinate

Vehicles are spawned at position specified in settings in global NED
*/

class NedTransform
{
public:
	typedef msr::airlib::Vector3r Vector3r;
	typedef msr::airlib::Quaternionr Quaternionr;
	typedef msr::airlib::Pose Pose;

public:
	NedTransform(const AirSimUnity::UnityTransform& global_transform);
	NedTransform(const NedTransform& global_transform);

private:
	AirSimUnity::UnityTransform global_transform_;
	AirSimUnity::AirSimVector local_ned_offset_;
};

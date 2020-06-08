#pragma once 

#include "Vehicles/Multirotor/MultirotorPawnEvents.h"
#include "AirSimStructs.hpp"

namespace UnityUtilities
{
	static AirSimUnity::RotorInfo Convert_to_UnityRotorInfo(const MultirotorPawnEvents::RotorActuatorInfo& rotor_info)
	{
		AirSimUnity::RotorInfo unityRotorInfo;
		unityRotorInfo.rotor_control_filtered = rotor_info.rotor_control_filtered;
		unityRotorInfo.rotor_direction = rotor_info.rotor_direction;
		unityRotorInfo.rotor_speed = rotor_info.rotor_speed;
		unityRotorInfo.rotor_thrust = rotor_info.rotor_thrust;
		return unityRotorInfo;
	}

	static msr::airlib::Vector3r Convert_to_Vector3r(const AirSimUnity::AirSimVector& airSimVector)
	{
		msr::airlib::Vector3r result(airSimVector.x, airSimVector.y, airSimVector.z);
		return result;
	}

	static AirSimUnity::AirSimVector Convert_to_AirSimVector(const msr::airlib::Vector3r vec)
	{
		AirSimUnity::AirSimVector result(vec.x(), vec.y(), vec.z());
		return result;
	}


	static msr::airlib::CollisionInfo Convert_to_AirSimCollisioinInfo(const AirSimUnity::AirSimCollisionInfo& collision_info)
	{
		msr::airlib::CollisionInfo collisionInfo;
		collisionInfo.collision_count = collision_info.collision_count;
		collisionInfo.has_collided = collision_info.has_collided;
		collisionInfo.impact_point = Convert_to_Vector3r(collision_info.impact_point);
		collisionInfo.normal = Convert_to_Vector3r(collision_info.normal);
		collisionInfo.object_id = collision_info.object_id;
		collisionInfo.object_name = collision_info.object_name;
		collisionInfo.penetration_depth = collision_info.penetration_depth;
		collisionInfo.position = Convert_to_Vector3r(collision_info.position);
		collisionInfo.time_stamp = collision_info.time_stamp;

		return collisionInfo;
	}

	static AirSimUnity::AirSimImageRequest Convert_to_UnityRequest(const msr::airlib::ImageCaptureBase::ImageRequest &request)
	{
		AirSimUnity::AirSimImageRequest airsim_request;
		airsim_request.camera_name = const_cast<char*>(request.camera_name.c_str());
		airsim_request.compress = request.compress;
		airsim_request.image_type = request.image_type;
		airsim_request.pixels_as_float = airsim_request.pixels_as_float;
		return airsim_request;
	}

	static void Convert_to_AirsimResponse(const AirSimUnity::AirSimImageResponse &src, msr::airlib::ImageCaptureBase::ImageResponse &dest, std::string cameraName)
	{
		//TODO, we need to get this cameraName from the source itself rather than having it as a seperate parameter which ultemately get it from Request.
		dest.camera_name = cameraName;
		dest.camera_position.x() = src.camera_position.x;
		dest.camera_position.y() = src.camera_position.y;
		dest.camera_position.z() = src.camera_position.z;
		dest.camera_orientation.x() = src.camera_orientation.x;
		dest.camera_orientation.y() = src.camera_orientation.y;
		dest.camera_orientation.z() = src.camera_orientation.z;
		dest.camera_orientation.w() = src.camera_orientation.w;
		dest.time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (src.image_uint_len == 0 && src.image_float_len)
		{
			dest.message = "no image capture";
		}
		else
		{
			dest.message = "success";
		}
		dest.pixels_as_float = src.pixels_as_float;
		dest.compress = src.compress;
		dest.width = src.width;
		dest.height = src.height;
		dest.image_type = src.image_type;
		dest.image_data_float.clear();
		dest.image_data_uint8.clear();
		for (int i = 0; i < src.image_uint_len; i++)
		{
			dest.image_data_uint8.push_back(static_cast<unsigned char>(src.image_data_uint[i]));
		}
		for (int i = 0; i < src.image_float_len; i++)
		{
			dest.image_data_float.push_back(src.image_data_float[i]);
		}
	}

	static msr::airlib::Pose Convert_to_Pose(const AirSimUnity::AirSimPose& airSimPose)
	{
		msr::airlib::Pose pose = msr::airlib::Pose();
		pose.position.x() = airSimPose.position.x;
		pose.position.y() = airSimPose.position.y;
		pose.position.z() = airSimPose.position.z;
		pose.orientation.x() = airSimPose.orientation.x;
		pose.orientation.y() = airSimPose.orientation.y;
		pose.orientation.z() = airSimPose.orientation.z;
		pose.orientation.w() = airSimPose.orientation.w;
		return pose;
	}

	static AirSimUnity::AirSimPose Convert_to_AirSimPose(const msr::airlib::Pose& pose)
	{
		AirSimUnity::AirSimPose airSimPose = AirSimUnity::AirSimPose();
		airSimPose.position.x = pose.position.x();
		airSimPose.position.y = pose.position.y();
		airSimPose.position.z = pose.position.z();
		airSimPose.orientation.x = pose.orientation.x();
		airSimPose.orientation.y = pose.orientation.y();
		airSimPose.orientation.z = pose.orientation.z();
		airSimPose.orientation.w = pose.orientation.w();
		return airSimPose;
	}
}

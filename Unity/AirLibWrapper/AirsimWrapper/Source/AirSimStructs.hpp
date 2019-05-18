#pragma once

#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"
#include "common/ImageCaptureBase.hpp"

/*
* This file contains structures that facilitates the data transfer between
* the C++(AirLib) layer and the C#(Unity) layer.
* Equivalent C# structures are maintained in Unity and these two must be in sync.
*/

namespace AirSimUnity
{
	enum ErrorLogSeverity
	{
		Information = 0,
		Warnning = 1,
		Error = 2
	};

	struct AirSimVector
	{
		float x;
		float y;
		float z;

		AirSimVector() : x(0), y(0), z(0) {}
		AirSimVector(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
		AirSimVector(msr::airlib::Vector3r vec)
		{
			x = vec.x();
			y = vec.y();
			z = vec.z();
		}

		AirSimVector operator+(const AirSimVector& V) const
		{
			return AirSimVector(x + V.x, y + V.y, z + V.z);
		}

		AirSimVector operator-(const AirSimVector& V) const
		{
			return AirSimVector(x - V.x, y - V.y, z - V.z);
		}
	};

	struct AirSimQuaternion
	{
		float x;
		float y;
		float z;
		float w;

		AirSimQuaternion() : x(0), y(0), z(0), w(0) {}
		AirSimQuaternion(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
		AirSimQuaternion(msr::airlib::Quaternionr quat)
		{
			x = quat.x();
			y = quat.y();
			z = quat.z();
			w = quat.w();
		}
	};

	struct AirSimPose
	{
		AirSimVector position;
		AirSimQuaternion orientation;

		AirSimPose()
		{
			position = AirSimVector();
			orientation = AirSimQuaternion();
		}

		AirSimPose(msr::airlib::Pose pose)
		{
			position = pose.position;
			orientation = pose.orientation;
		}

		AirSimPose(AirSimVector in_position, AirSimQuaternion in_orientation)
		{
			position = in_position;
			orientation = in_orientation;
		}
	};

	struct AirSimTwist
	{
		AirSimVector angular;
		AirSimVector linear;
	};

	struct AirSimAccelerations
	{
		AirSimVector angular;
		AirSimVector linear;
	};

	struct AirSimRCData
	{
		#ifdef _WIN32
			_int64 timestamp = 0;
		#elif defined(__linux__) || defined(__APPLE__)
			int64_t timestamp = 0;
		#endif
		float pitch = 0, roll = 0, throttle = 0, yaw = 0;
		float left_z = 0, right_z = 0;
		unsigned int  switch1 = 0, switch2 = 0, switch3 = 0, switch4 = 0,
			switch5 = 0, switch6 = 0, switch7 = 0, switch8 = 0;
		bool is_initialized = false; //is RC connected?
		bool is_valid = false; //must be true for data to be valid
	};

	struct AirSimCollisionInfo
	{
		bool has_collided = false;;
		AirSimVector normal;
		AirSimVector impact_point;
		AirSimVector position;
		float penetration_depth = 0.0f;
		#ifdef _WIN32
			_int64 time_stamp = 0;
		#elif defined(__linux__) || defined(__APPLE__)
			int64_t time_stamp = 0;
		#endif
		int collision_count = 0;
		char* object_name;
		int object_id = -1;
	};

	struct AirSimKinematicState
	{
		AirSimPose pose;
		AirSimTwist twist;
		AirSimAccelerations accelerations;
	};

	struct AirSimImageRequest {
		char* camera_name;
		msr::airlib::ImageCaptureBase::ImageType image_type = static_cast<msr::airlib::ImageCaptureBase::ImageType>(0);
		bool pixels_as_float = false;
		bool compress = false;
	};

	struct AirSimImageResponse {
		int image_uint_len = 0;
		unsigned char* image_data_uint;
		int image_float_len = 0;
		float* image_data_float;
		AirSimVector camera_position;
		AirSimQuaternion camera_orientation;
		bool pixels_as_float = false;
		bool compress = false;
		int width = 0;
		int height = 0;
		msr::airlib::ImageCaptureBase::ImageType image_type = static_cast<msr::airlib::ImageCaptureBase::ImageType>(0);
	};

	struct RotorInfo
	{
		msr::airlib::real_T rotor_speed = 0;
		int rotor_direction = 0;
		msr::airlib::real_T rotor_thrust = 0;
		msr::airlib::real_T rotor_control_filtered = 0;
	};

	struct AirSimCarState
	{
		int gear = 0;
		float speed = 0.0f;
		#ifdef _WIN32
			__int64 time_stamp = 0;
		#elif defined(__linux__) || defined(__APPLE__)
			int64_t timestamp = 0;
		#endif
		
		float engineMaxRotationSpeed = 0;
		float engineRotationSpeed = 0;
		AirSimPose pose;
	};

	struct AirSimCameraInfo
	{
		AirSimPose pose;
		float fov = 90.0f;
	};

	struct UnityTransform
	{
	public:
		/** Rotation of this transformation, as a quaternion */
		AirSimQuaternion Rotation;
		/** Translation of this transformation, as a vector */
		AirSimVector Position;
		/** 3D scale (always applied in local space) as a vector */
		AirSimVector Scale3D;

	public:

		UnityTransform()
		{
			Rotation = AirSimQuaternion(0, 0, 0, 1);
			Position = AirSimVector(0, 0, 0);
			Scale3D = AirSimVector(1, 1, 1);
		}

		UnityTransform(const UnityTransform& obj) :
			Rotation(obj.Rotation),
			Position(obj.Position),
			Scale3D(obj.Scale3D)
		{}
	};

	struct RayCastHitResult
	{
		bool isHit = false;
		float distance = 0.0f;
	};
}
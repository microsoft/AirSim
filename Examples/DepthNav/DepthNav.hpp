// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

// includes needed to call RPC APIs
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

//includes for vector math and other common types
#include "common/Common.hpp"
#include <exception>

#include "../../SGM/src/sgmstereo/sgmstereo.h"
#include "../../SGM/src/stereoPipeline/StateStereo.h"                                                                                        
namespace msr { namespace airlib {

class DepthNav {
public: //types
	struct Params {
		//Camera FOV
		real_T fov = Utils::degreesToRadians(90.0f);

		//When goal is outside of frustum, we need to rotate
		//below specifies max angle per step
		real_T rotation_step_limit = Utils::degreesToRadians(5.0f);

        //depth image dimension, index of pixel x,y = x*width + y 
		unsigned int depth_width, depth_height;

		//Number of cells the depth image gets divided in to. Each cell is square.
		unsigned int M, N;

		//In a cell in depth image, what is the distance we consider for a pixel
		//that would qualify it as an obstacle that needs to be avoided
		//TODO: we should use current velocity to determine this
		real_T max_allowed_obs_dist = 5; //in meters

		//Vehicle dimensions
		real_T margin_w = 1.25f, margin_h = 2.0f;
		real_T vehicle_width = 0.98f * margin_w, vehicle_height = 0.26f * margin_h;
		unsigned int vehicle_width_px, vehicle_height_px;

        real_T min_exit_dist_from_goal = 1.0f;

        real_T control_loop_period = 0.25f*max_allowed_obs_dist; //30.0f / 1000; //sec
        real_T max_linear_speed = 10; // m/s
        real_T max_angular_speed = 6; // rad/s
	};

    class DepthNavException : public std::runtime_error {
    public:
        DepthNavException(const std::string& message)
            : std::runtime_error(message)
        {}
    };

public:
	Params params_;

	DepthNav(const Params& params = Params())
		: params_(params)
	{}

    void initialize(RpcLibClientBase& client, const std::vector<ImageCaptureBase::ImageRequest>& request){
        const std::vector<ImageCaptureBase::ImageResponse>& response_init = client.simGetImages(request);
        params_.depth_width = response_init.at(0).width;
        params_.depth_height = response_init.at(0).height;
		params_.vehicle_height_px = int(ceil(params_.depth_height * params_.vehicle_height / (tan(params_.fov / 2) * params_.max_allowed_obs_dist * 2))); //height
		params_.vehicle_width_px = int(ceil(params_.depth_width * params_.vehicle_width / (tan(hfov2vfov(params_.fov, params_.depth_height, params_.depth_width) / 2) * params_.max_allowed_obs_dist * 2))); //width    
    }

    virtual void gotoGoal(const Pose& goal_pose, RpcLibClientBase& client, const std::vector<ImageCaptureBase::ImageRequest>& request)
    {

        typedef ImageCaptureBase::ImageResponse ImageResponse;

        do {
            const Pose current_pose = client.simGetVehiclePose();

            const std::vector<ImageResponse>& response = client.simGetImages(request);

            if (response.size() == 0)
                throw std::length_error("No images received!");

            const Pose next_pose = getNextPose(response.at(0).image_data_float, goal_pose.position, 
                current_pose, params_.control_loop_period);

            if (VectorMath::hasNan(next_pose))
                throw DepthNavException("No further path can be found.");
            else { 
                client.simSetVehiclePose(next_pose, true);
            /*
                //convert pose to velocity commands
                //obey max linear speed constraint
                Vector3r linear_vel = (next_pose.position - current_pose.position) / params_.control_loop_period;
                if (linear_vel.norm() > params_.max_linear_speed) {
                    linear_vel = linear_vel.normalized() * params_.max_linear_speed;
                }

                //obey max angular speed constraint
                Quaternionr to_orientation = next_pose.orientation;
                Vector3r angular_vel = VectorMath::toAngularVelocity(current_pose.orientation,
                    next_pose.orientation, params_.control_loop_period);
                real_T angular_vel_norm = angular_vel.norm();
                if (angular_vel_norm > params_.max_angular_speed) {
                    real_T slerp_alpha = params_.max_angular_speed / angular_vel_norm;
                    to_orientation = VectorMath::slerp(current_pose.orientation, to_orientation, slerp_alpha);
                }

                //Now we can use (linear_vel, to_orientation) for vehicle commands
                
                //For ComputerVision mode, we will just create new pose
                Pose contrained_next_pose(current_pose.position + linear_vel * params_.control_loop_period,
                    to_orientation);
                client.simSetVehiclePose(contrained_next_pose, true);
            */  
            }

            real_T dist2goal = getDistanceToGoal(next_pose.position, goal_pose.position);
            if (dist2goal <= params_.min_exit_dist_from_goal)
                return;
            Utils::log(Utils::stringf("Distance to target: %f", dist2goal));

        } while (true);
    }



    virtual void gotoGoalSGM(const Pose& goal_pose, RpcLibClientBase& client, const std::vector<ImageCaptureBase::ImageRequest>& request, CStateStereo * p_state)
    {

        typedef ImageCaptureBase::ImageResponse ImageResponse;
        typedef common_utils::FileSystem FileSystem;

        float dtime = 0;
        int counter = 0;

        std::vector<float> sgm_depth_image(params_.depth_height*params_.depth_width);

        do {
            const Pose current_pose = client.simGetVehiclePose();

            const std::vector<ImageResponse>& response = client.simGetImages(request);

            if (response.size() == 0)
                throw std::length_error("No images received!");
            /*
            if (response.at(0).height != params_.depth_height || response.at(0).width != params_.depth_height)
                throw DepthNavException("Image Dimension mismatch. Please check left camera in the AirSim config file.");

            if (response.at(1).height != params_.depth_height || response.at(1).width != params_.depth_height)
                throw DepthNavException("Image Dimension mismatch. Please check right camera in the AirSim config file.");
            */
            const std::vector<uint8_t>& left_image = response.at(0).image_data_uint8;
            const std::vector<uint8_t>& right_image = response.at(1).image_data_uint8;

            //baseline * focal_length = depth * disparity
            float f = params_.depth_width / (2 * tan(params_.fov/2));
            float B = 0.25; 
   
            p_state->ProcessFrameAirSim(counter, dtime, left_image, right_image);
                                                                     
	        for (unsigned int idx = 0; idx < (params_.depth_height*params_.depth_width); idx++)
	        {
		        float d = p_state->dispMap[idx];
		        if (d < FLT_MAX)
		        {
                    //float dn = (d - dmin)/drange;
                    sgm_depth_image[idx] = -(B*f/d);

		        }
	        }

            counter++;

            const Pose next_pose = getNextPose(sgm_depth_image, goal_pose.position, 
                current_pose, params_.control_loop_period);

            if (VectorMath::hasNan(next_pose))
                throw DepthNavException("No further path can be found.");
            else { 
                client.simSetVehiclePose(next_pose, true);
            /*
                //convert pose to velocity commands
                //obey max linear speed constraint
                Vector3r linear_vel = (next_pose.position - current_pose.position) / params_.control_loop_period;
                if (linear_vel.norm() > params_.max_linear_speed) {
                    linear_vel = linear_vel.normalized() * params_.max_linear_speed;
                }

                //obey max angular speed constraint
                Quaternionr to_orientation = next_pose.orientation;
                Vector3r angular_vel = VectorMath::toAngularVelocity(current_pose.orientation,
                    next_pose.orientation, params_.control_loop_period);
                real_T angular_vel_norm = angular_vel.norm();
                if (angular_vel_norm > params_.max_angular_speed) {
                    real_T slerp_alpha = params_.max_angular_speed / angular_vel_norm;
                    to_orientation = VectorMath::slerp(current_pose.orientation, to_orientation, slerp_alpha);
                }

                //Now we can use (linear_vel, to_orientation) for vehicle commands
                
                //For ComputerVision mode, we will just create new pose
                Pose contrained_next_pose(current_pose.position + linear_vel * params_.control_loop_period,
                    to_orientation);
                client.simSetVehiclePose(contrained_next_pose, true);
            */  
            }

            real_T dist2goal = getDistanceToGoal(next_pose.position, goal_pose.position);
            if (dist2goal <= params_.min_exit_dist_from_goal)
                return;
            Utils::log(Utils::stringf("Distance to target: %f", dist2goal));

        } while (true);
    }

protected:    
	/* 
    depth_image is 2D float array for which width and height are specified in Params
	goal is specified in world frame and typically provided by the global planner
	current_pose is current pose of the vehicle in world frame
	dt is time passed since last call in seconds
	return next pose that vehicle should be in
	Pose getNextPose(const real_T ** const depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
    */
	virtual Pose getNextPose(const std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt) = 0;

	/*
	Input:
	*ray = ray from origin to goal
	*planeNormal = normal of the plane
	*max_allowed_obs_dist = obstacle distance threshold
	Output:
	*intersection_point = the intersection point on the plane in body frame
	*/
    virtual Vector3r linePlaneIntersection(const Vector3r& ray_n, const Vector3r& planeNormal, real_T dist)
    {
        if (planeNormal.dot(ray_n) < Utils::epsilon<real_T>()) {
            return VectorMath::nanVector(); // No intersection, the line is parallel to the plane or behind
        }

        // Compute the intersection point on the plane
        real_T x = dist;
        real_T y = dist * ray_n.y() / ray_n.x();
        real_T z = dist * ray_n.z() / ray_n.x();
        // output contact point
        return Vector3r(x, y, z);
    }

	//convert horizontal fov to vertical fov
	real_T hfov2vfov(real_T hfov, unsigned int img_h, unsigned int img_w)
	{
		real_T aspect = real_T(img_h) / real_T(img_w);
		real_T vfov = 2 * std::atan(std::tan(hfov / 2) * aspect);
		return vfov;
	}

	/*
	*https://www.edmundoptics.com/resources/application-notes/imaging/understanding-focal-length-and-field-of-view/
	*/
	Vector2r getPlaneSize(real_T distance, real_T hfov, real_T vfov) {
		real_T height_world = 2 * distance * std::tan(vfov / 2);
		real_T width_world = 2 * distance * std::tan(hfov / 2);
		return Vector2r(height_world, width_world);
	}

	std::vector<int> spiralOrder(int m, int n, int idx) {
		std::vector<int> spiral_idx;
		enum Dirs_ { right, down, left, up };

		if (0 == m || 0 == n) return spiral_idx;
		int r = idx / m, c = idx - r * m, rmin = r - 1, rmax = r + 1, cmin = c - 1, cmax = c + 1;

		int cnt = m * n;
		int dx[4] = { 1, 0, -1,  0 };
		int dy[4] = { 0, 1,  0, -1 };
		int current_idx = idx;

		Dirs_ dir = right;
		int i = 0;
		while (i < cnt) {
			if (0 <= r && r < n && 0 <= c && c < m) { spiral_idx.push_back(current_idx); ++i; }
			r += dy[dir];
			c += dx[dir];
			current_idx = r * m + c;

			if (right == dir && c == cmax) { dir = down; cmax++; } //right
			else if (down == dir && r == rmax) { dir = left; rmax++; } //down
			else if (left == dir && c == cmin) { dir = up; cmin--; } //left
			else if (up == dir && r == rmin) { dir = right; rmin--; } //up
		}
		return spiral_idx;
	}

	//Returns index of nearest neighbor
	unsigned int nearest_neighbor(std::vector<Vector2r> arr, Vector2r query) {
		real_T min_dist = static_cast<real_T>(Utils::max<uint16_t>());
		unsigned int index = 0;
		for (unsigned int i = 0; i < arr.size(); i++) {
			Vector2r diff = arr[i] - query;
			real_T dist = std::sqrt(diff.dot(diff));
			if (dist < min_dist) {
				min_dist = dist;
				index = i;
			}
		}
		return index;
	}

	Pose rotateToGoal(Pose current_pose, Vector3r goal) {

		Quaternionr fromQuat = current_pose.orientation;
		//get rotation we need
		Quaternionr toQuat = VectorMath::lookAt(current_pose.position, goal);
		//Remove roll component
		//toQuat = VectorMath::toQuaternion(VectorMath::getPitch(toQuat), 0, VectorMath::getYaw(toQuat));

		//Compute angle between quats
		Quaternionr diffQuat = VectorMath::coordOrientationSubtract(toQuat, fromQuat);
		real_T diffAngle = 2 * std::acos(diffQuat.w());
		real_T slerp_alpha = 0;
		if (diffAngle > 0) { slerp_alpha = params_.rotation_step_limit / diffAngle; }
		if (slerp_alpha > 1) { slerp_alpha = 1; }

		//using spherical interpolation compute fraction of quaternion
		Quaternionr stepQuat = VectorMath::slerp(fromQuat, toQuat, slerp_alpha);

		//add fraction of quaternion to current orientation
		return Pose(current_pose.position, stepQuat.normalized());
	}

	std::vector<Vector2r> getCellCenters() {
		params_.M = params_.depth_height / params_.vehicle_height_px;
		params_.N = params_.depth_width / params_.vehicle_width_px;
		unsigned int M_offset = params_.depth_height - params_.vehicle_height_px * params_.M;
		unsigned int N_offset = params_.depth_width - params_.vehicle_width_px * params_.N;
		std::vector<Vector2r> cell_centers;
		Vector2r cell_center;

		//Leave one cell free at boundaries
		for (unsigned int i = 0; i < params_.M; i++) {
			for (unsigned int j = 0; j < params_.N; j++) {
				cell_center.x() = real_T(j*params_.vehicle_width_px + 0.5f * (params_.vehicle_width_px + N_offset));
				cell_center.y() = real_T(i*params_.vehicle_height_px + 0.5f * (params_.vehicle_height_px + M_offset));
				cell_centers.push_back(cell_center);
			}
		}
		return cell_centers;
	}

	real_T getDistanceToGoal(Vector3r current_position, Vector3r goal)
	{
		Vector3r goalVec = goal - current_position;
		return goalVec.norm();
	}

	//returns true if goal in body frame in within frustum
	bool isInFrustrum(const Vector3r& goal_body)
	{
		//for simplicity, assume frustum is circular
		//front is +X so get the dot product of unit +X with goal_body
		//if dot product < pre-calculated value then goal_body is outside
		//frustum
        
        real_T angle = VectorMath::angleBetween(VectorMath::front(), goal_body.normalized(), true);
        return std::abs(angle) <= params_.fov;
	}
};

}}
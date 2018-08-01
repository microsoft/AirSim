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

namespace msr { namespace airlib {

	class DepthNav {
	public: //types
		struct Params {
			//Camera FOV
			real_T fov = Utils::degreesToRadians(90.0f);
			//min dot product required for goal vector to be
			//considered so goal is inside frustum
			real_T min_frustrum_alignment = (1.0f + 0.75f) / 2;

			//When goal is outside of frustum, we need to rotate
			//below specifies max angle per step
			real_T rotation_step_limit = Utils::degreesToRadians(5.0f);;

			unsigned int depth_width = 256, depth_height = 144;

			//Number of cells the depth image gets divided in to. Each cell is square.
			unsigned int M, N;

			//In a cell in depth image, what is the distance we consider for a pixel
			//that would qualify it as an obstacle that needs to be avoided
			//TODO: we should use current velocity to determine this
			real_T max_allowed_obs_dist = 5; //in meters

			//In a cell in depth image, how many pixels should be closer than max_allowed_obs_dist
			//to consider that cell to be avoided. Otherwise we consider that cell to be free
			unsigned int max_allowed_obs_per_block = 1;

			//Number of free cells for width and height required for the vehicle to pass through
			unsigned int req_free_width = 40, req_free_height = 10;

			//Vehicle dimensions
			float margin = 1.2f;
			float vehicle_width = 0.98f * margin, vehicle_height = 0.26f * margin;
			unsigned int vehicle_width_px, vehicle_height_px;

		};

	public:
		DepthNav(const Params& params = Params())
			: params_(params)
		{
			params_.vehicle_height_px = int(ceil(params_.depth_height * params_.vehicle_height / (tan(params_.fov / 2) * params_.max_allowed_obs_dist * 2))); //height
			params_.vehicle_width_px = int(ceil(params_.depth_width * params_.vehicle_width / (tan(hfov2vfov(params_.fov, params_.depth_height, params_.depth_width) / 2) * params_.max_allowed_obs_dist * 2))); //width
		}

		//depth_image is 2D float array for which width and height are specified in Params
		//goal is specified in world frame and typically provided by the global planner
		//current_pose is current pose of the vehicle in world frame
		//dt is time passed since last call in seconds
		//return next pose that vehicle should be in
		//Pose getNextPose(const real_T ** const depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
		Pose getNextPose(std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
		{
			auto goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);
			//Vector3r goal_body = goal - current_pose.position;
			if (isInFrustrum(goal_body, params_.fov)) {
				/*
				Note: It would be good to have an individual function for many of below steps
				so we can test it individual from bottom up and improve each sub-algorithm
				*/
				//1. Let's have a plane that fits in our frustum at x = 1 (remember +X is front, +Y is right in NED)
				Vector2r planeSize = getPlaneSize(params_.max_allowed_obs_dist, params_.fov, hfov2vfov(params_.fov, params_.depth_height, params_.depth_width));
				//2. We will compute x_min, y_min, x_max, y_max for this plane in body frame.
				float z_min = -planeSize.x() / 2;
				float y_min = -planeSize.y() / 2;
				float z_max = planeSize.x() / 2;
				float y_max = planeSize.y() / 2;
				//3. Then we will compute x_goal,y_goal where the vector goal_body intersects this plane.
				Vector3r intersect_point = linePlaneIntersection(goal_body, VectorMath::front(), params_.max_allowed_obs_dist);
				//Check if intersection is valid
				if (VectorMath::hasNan(intersect_point))
				{
					return rotateToGoal(current_pose, goal);
				}
				else if (intersect_point.z() < z_min || intersect_point.z() > z_max || intersect_point.y() < y_min || intersect_point.y() > y_max)
				{
					return rotateToGoal(current_pose, goal);
				}
				else
				{
					//4. So now we have a rectangle and a point within it
					//5. Then descretize the rectangle in M * N cells. This is simply truncation after division. For each pixel we can now get cell coordinates.
					std::vector<Vector2r> cell_centers = getCellCenters();
					//6. Compute cell coordinates i, j for x_goal and y_goal.
					float y_px = intersect_point.y() * params_.depth_width / planeSize.y() + params_.depth_width / 2;
					float z_px = intersect_point.z() * params_.depth_height / planeSize.x() + params_.depth_height / 2;
					//Get nearest neighbor to goal
					unsigned int cell_idx = nearest_neighbor(cell_centers, Vector2r(y_px, z_px));
					//Get spiral indexes
					std::vector<int> spiral_idxs = spiralOrder(params_.M, params_.N, cell_idx);
					/*7. Until free space is found
					For p = -params.req_free_width to +params.req_free_width
					For q = -params.req_free_height to +params.req_free_height
					Query block (i + p, j + q) in depth image to see if it is free
					We can use params.max_allowed_obs_per_block and params.max_allowed_obs_dist
					Make sure to cache result of the query for each block
					If all blocks in above for-for loop where not free
					then move i,j spirally within the rectangle (or do something more simple?)
					If all blocks have been marked as occupied
					then return Pose::nanPose() indicating no more moves possible
					*/
					Vector2r goal_px;
					for (int i = 0; i < cell_centers.size(); ++i) {
						if (isCellFree(depth_image, cell_centers[spiral_idxs[i]])) {
							//8. We are here if we have found cell coordinates i, j as center of the free window from step #7
							//9. Compute i_x_center, j_y_center that would be center pixel of this cell in the plane for x = 1
							goal_px = cell_centers[spiral_idxs[i]];

							////Body frame in meters
							float goal_y = (goal_px.x() - params_.depth_width / 2) * planeSize.y() / params_.depth_width;
							float goal_z = (goal_px.y() - params_.depth_height / 2) * planeSize.x() / params_.depth_height;
							Vector3r goal_world = VectorMath::transformToWorldFrame(Vector3r(params_.max_allowed_obs_dist, goal_y, goal_z).normalized(), current_pose);

							Pose goal_pose = current_pose;
							//10. Compute next orientation with similar algorithm as in else section below
							goal_pose = rotateToGoal(current_pose, goal_world);

							//11. Compute next position along vector (i_x_center, j_y_center, 1) and transform it to world frame
							goal_pose.position = VectorMath::transformToWorldFrame(Vector3r(params_.max_allowed_obs_dist, goal_y, goal_z).normalized()*dt, goal_pose);

							//12. return pose using result from step 10 and 11
							return goal_pose;
						}
					}
					//If no free space was found
					return Pose::nanPose();
				}
			}
			//Not in frustrum
			else {
				return Pose::nanPose();
			}
		}

		/*
		Input:
		*ray = ray from origin to goal
		*planeNormal = normal of the plane
		*max_allowed_obs_dist = obstacle distance threshold
		Output:
		*intersection_point = the intersection point on the plane in body frame
		*/
		Vector3r linePlaneIntersection(Vector3r ray, Vector3r planeNormal, float max_allowed_obs_dist) {

			if (planeNormal.dot(ray) < FLT_MIN) {
				return VectorMath::nanVector(); // No intersection, the line is parallel to the plane or behind
			}

			// Compute the intersection point on the plane
			float x = max_allowed_obs_dist;
			float y = max_allowed_obs_dist * ray.y() / ray.x();
			float z = max_allowed_obs_dist * ray.z() / ray.x();
			// output contact point
			return Vector3r(x, y, z);
		}

		//convert horizonal fov to vertical fov
		float hfov2vfov(float hfov, unsigned int img_h, unsigned int img_w)
		{
			float aspect = float(img_h) / float(img_w);
			float vfov = 2 * atan(tan(hfov / 2) * aspect);
			return vfov;
		}

		/*
		*https://www.edmundoptics.com/resources/application-notes/imaging/understanding-focal-length-and-field-of-view/
		*/
		Vector2r getPlaneSize(float distance, float hfov, float vfov) {
			float height_world = 2 * distance * tanf(vfov / 2);
			float width_world = 2 * distance * tanf(hfov / 2);
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
			float min_dist = UINT16_MAX;
			unsigned int index = 0;
			for (unsigned int i = 0; i < arr.size(); i++) {
				Vector2r diff = arr[i] - query;
				float dist = sqrt(diff.dot(diff));
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
			float diffAngle = 2 * std::acosf(diffQuat.w());
			float slerp_alpha = 0;
			if (diffAngle > 0) { slerp_alpha = params_.rotation_step_limit / diffAngle; }
			if (slerp_alpha > 1) { slerp_alpha = 1; }

			//using spherical interpolation compute fraction of quaternion
			Quaternionr stepQuat = VectorMath::slerp(fromQuat, toQuat, slerp_alpha);

			//add fraction of quaternion to current orientation
			return Pose(current_pose.position, stepQuat.normalized());
		}

		bool isCellFree(std::vector<float>& img, Vector2r cell_center) {

			//std::vector<float> crop;
			unsigned int counter = 0;

			for (int i = int(cell_center.y() - params_.vehicle_height_px / 2); i < int(cell_center.y() + params_.vehicle_height_px / 2); i++) {
				for (int j = int(cell_center.x() - params_.vehicle_width_px / 2); j<int(cell_center.x() + params_.vehicle_width_px / 2); j++) {
					int idx = i * params_.depth_width + j;
					//crop.push_back(img[idx]);
					if (img[idx] < params_.max_allowed_obs_dist) {
						counter++;
					}
				}
			}

			if (counter > params_.max_allowed_obs_per_block)
			{
				return false;
			}
			else
			{
				return true;
			}

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
					cell_center.x() = float(j*params_.vehicle_width_px + 0.5f * (params_.vehicle_width_px + N_offset));
					cell_center.y() = float(i*params_.vehicle_height_px + 0.5f * (params_.vehicle_height_px + M_offset));
					cell_centers.push_back(cell_center);
				}
			}
			return cell_centers;
		}

		float getDistanceToGoal(Vector3r current_position, Vector3r goal)
		{
			Vector3r goalVec = goal - current_position;
			return goalVec.norm();
		}

		//returns true if goal in body frame in within frustum
		bool isInFrustrum(const Vector3r& goal_body, real_T fov)
		{
			//for simplicity, assume frustum is circular
			//front is +X so get the dot product of unit +X with goal_body
			//if dot product < pre-calculated value then goal_body is outside
			//frustum
			return true;
		}

	private:
		Params params_;
	};
}
}
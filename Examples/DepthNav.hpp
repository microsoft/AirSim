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


namespace msr {
	namespace airlib {



		class DepthNav {

		public: //types

			struct Params {

				//Camera FOV
				float fov = Utils::degreesToRadians(90.0f);

				//min dot product required for goal vector to be
				//considered so goal is inside frustum
				float min_frustrum_alignment = (1.0f + 0.75f) / 2;

				//Define start and goal poses
				Pose startPose = Pose(Vector3r(0, 5, -1), Quaternionr(0, 0, 0, 0)); //start pose
				Pose goalPose = Pose(Vector3r(120, 0, -1), Quaternionr(0, 0, 0, 0)); //final pose

			};



		public:

			DepthNav(const Params& params = Params())

				: params_(params)

			{

			}
			
			/*
			*Adapted from http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final
			*Build a unit quaternion representing the rotation
			* from u to v. The input vectors need not be normalised.
			*/
			Quaternionr getQuatBetweenVecs(Vector3r u, Vector3r v)
			{

				float norm_u_norm_v = sqrt(u.dot(u) * v.dot(v));
				float real_part = norm_u_norm_v + u.dot(v);
				Vector3r w;

				if (real_part < 1.e-6f * norm_u_norm_v)
				{
					/* If u and v are exactly opposite, rotate 180 degrees
					* around an arbitrary orthogonal axis. Axis normalisation
					* can happen later, when we normalise the quaternion. */
					real_part = 0.0f;
					//w = abs(u.x()) > abs(u.z()) ? Vector3r(-u.y(), u.x(), 0.f) : Vector3r(0.f, -u.z(), u.y());
					w = abs(u.x()) > abs(u.z()) ? Vector3r(0.f, -u.y(), u.x()) : Vector3r(-u.z(), u.y(), 0.f);
				}
				else
				{
					/* Otherwise, build quaternion the standard way. */
					w = u.cross(v);
				}

				Quaternionr q(real_part, w.x(), w.y(), w.z());

				return q.normalized();
			}

			float getNorm2(Vector3r u)
			{		
				return sqrt(u.dot(u));
			}

			//compute bounding box size
			Vector2r compute_bb(Vector2r image_sz, Vector2r obj_sz, float hfov, float distance)
			{
				float vfov = hfov2vfov(hfov, image_sz);
				Vector2r box;
				box.x() = ceil(obj_sz.x() * image_sz.x() / (tan(hfov / 2)*distance * 2)); //height
				box.y() = ceil(obj_sz.y() * image_sz.y() / (tan(vfov / 2)*distance * 2)); //width
				return box;
			}

			//convert horizonal fov to vertical fov
			float hfov2vfov(float hfov, Vector2r image_sz)
			{
				float aspect = image_sz.x() / image_sz.y();
				float vfov = 2 * atan(tan(hfov / 2) * aspect);
				return vfov;
			}

			//goal is specified in world frame and typically provided by the global planner

			//current_pose is current pose of the vehicle in world frame

			//return next pose that vehicle should be in

			
			Pose getNextPose(const Pose& goal_pose, const Pose& current_pose)

			{

				//get goal in body frame

				auto goal_body = VectorMath::transformToBodyFrame(goal_pose.position, current_pose.orientation);



				//is goal inside frustum?

				float frustrum_alignment = std::fabsf(goal_body.dot(VectorMath::front()));

				if (frustrum_alignment >= params_.min_frustrum_alignment) {


				}

				else {

					//goal is not in the frustum. Let's rotate ourselves until we get 

					//goal in the frustum



				}

			}

/*
			def get_next_vec(self, depth, obj_sz, goal, pos) :

				[h, w] = np.shape(depth)
				[roi_h, roi_w] = compute_bb((h, w), obj_sz, self.hfov, self.coll_thres)

				# compute vector, distance and angle to goal
				t_vec, t_dist, t_angle = get_vec_dist_angle(np.array([goal.position.x_val, goal.position.y_val, goal.position.z_val]), np.array([pos.position.x_val, pos.position.y_val, pos.position.z_val]))

				# compute box of interest
				img2d_box = img2d[int((h - roi_h) / 2) : int((h + roi_h) / 2), int((w - roi_w) / 2) : int((w + roi_w) / 2)]

				# scale by weight matrix(optional)
				#img2d_box = np.multiply(img2d_box, w_mtx)

				# detect collision
				if (np.min(img2d_box) < coll_thres) :
					self.yaw = self.yaw - radians(self.limit_yaw)
				else:
			self.yaw = self.yaw + min(t_angle - self.yaw, radians(self.limit_yaw))

				pos.position.x_val = pos.position.x_val + self.step*cos(self.yaw)
				pos.position.y_val = pos.position.y_val + self.step*sin(self.yaw)

				return pos.position, self.yaw, t_dist
*/

		private:

			Params params_;
		};


		class DepthNavT {
		public: //types
			struct Params {
				//Camera FOV
				real_T fov = Utils::degreesToRadians(90.0f);
				//min dot product required for goal vector to be
				//considered so goal is inside frustum
				real_T min_frustrum_alighment = (1.0f + 0.75f) / 2;

				//When goal is outside of frustum, we need to rotate
				//below specifies how much increment to get to required rotation
				//TODO: we should slerp by max allowed rotation for dt
				real_T rotation_slerp_alpha = 0.5f;

				unsigned int depth_width = 300, depth_height = 300;

				//Number of cells the depth image gets divided in to. Each cell is square.
				unsigned int M, N;

				//In a cell in depth image, what is the distance we consider for a pixel
				//that would qualify it as an obstacle that needs to be avoided
				//TODO: we should use current velocity to determine this
				real_T max_allowed_obs_dist = 1; //in meters

												 //In a cell in depth image, how many pixels should be closer than max_allowed_obs_dist
												 //to consider that cell to be avoided. Otherwise we consider that cell to be free
				unsigned int max_allowed_obs_per_block = 1;

				//Number of free cells for width and height required for the vehicle to pass through
				unsigned int req_free_width = 3, req_free_height = 1;
			};

		public:
			DepthNavT(const Params& params = Params())
				: params_(params)
			{
			}

			//depth_image is 2D float array for which width and height are specified in Params
			//goal is specified in world frame and typically provided by the global planner
			//current_pose is current pose of the vehicle in world frame
			//dt is time passed since last call in seconds
			//return next pose that vehicle should be in
			Pose getNextPose(const real_T ** const depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
			{
				auto goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);
				if (isInFrustrum(goal_body, params_.fov)) {
					/*
					Note: It would be good to have an individual function for many of below steps
					so we can test it individual from bottom up and improve each sub-algorithm

					1. Let's have a plane that fits in our frustum at x = 1 (remember +X is front, +Y is right in NED)
					2. We will compute x_min, y_min, x_max, y_max for this plane in body frame.
					3. Then we will compute x_goal,y_goal where the vector goal_body intersects this plane.
					4. So now we have a rectangle and a point within it
					5. Then descretize the rectangle in M * N cells. This is simply truncation after division. For each pixel we can now get cell coordinates.
					6. Compute cell coordinates i, j for x_goal and y_goal.
					7. Until free space is found
					For p = -params.req_free_width to +params.req_free_width
					For q = -params.req_free_height to +params.req_free_height
					Query block (i + p, j + q) in depth image to see if it is free
					We can use params.max_allowed_obs_per_block and params.max_allowed_obs_dist
					Make sure to cache result of the query for each block
					If all blocks in above for-for loop where not free
					then move i,j spirally within the rectangle (or do something more simple?)
					If all blocks have been marked as occupied
					then return Pose::nanPose() indicating no more moves possible
					8. We are here if we have found cell coordinates i, j as center of the free window from step #7
					9. Compute i_x_center, j_y_center that would be center pixel of this cell in the plane for x = 1
					10. Compute next orientation with similar algorithm as in else section below
					11. Compute next position along vector (i_x_center, j_y_center, 1) and transform it to world frame
					12. return pose using result from step 10 and 11
					*/
				}
				else {
					//get rotation we need
					//TODO: below is toQuat to center but we only need somewhere at the edge of frustum
					Quaternionr toQuat = VectorMath::lookAt(current_pose.position, goal);
					Quaternionr fromQuat = current_pose.orientation;

					//using spherical interpolation compute fraction of quaternion
					Quaternionr stepQuat = VectorMath::slerp(fromQuat, toQuat, params_.rotation_slerp_alpha);

					//add fraction of quaternion to current orientation
					return Pose(current_pose.position,
						(current_pose.orientation * stepQuat).normalized());
				}
			}

			//returns true if goal in body frame in within frustum
			bool isInFrustrum(const Vector3r& goal_body, real_T fov)
			{
				//for simplicity, assume frustum is circular
				//front is +X so get the dot product of unit +X with goal_body
				//if dot product < pre-calculated value then goal_body is outside
				//frustum
			}

		private:
			Params params_;
		};
	}
}
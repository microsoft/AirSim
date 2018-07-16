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



	}
}
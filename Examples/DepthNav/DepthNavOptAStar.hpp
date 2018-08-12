#pragma once

#include "DepthNav.hpp"

namespace msr { namespace airlib {

class DepthNavOptAStar : public DepthNav {
protected:
    virtual Pose getNextPose(const std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt) override
    {
        Vector3r goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);

        if (!isInFrustrum(goal_body)) {
            Quaternionr rotate_to = VectorMath::lookAt(Vector3r::Zero(), goal_body);
            //Pose next_body_pose = (Vector3r::Zero(), rotate_to);
            return Pose(); // VectorMath::transformToWorldFrame()
        }

        Vector3r goal_body_n = goal_body.normalized();

        //find where goal ray intersects with plane
        rayToDepthPlaneIntersection(goal_body_n, 1);

        //get extents of the plane
        real_T vfov = hfov2vfov(params_.fov, params_.depth_height, params_.depth_width);
        Vector2r planeSize = getPlaneSize(1, params_.fov, vfov);
        //real_T z_min = -planeSize.x() / 2, z_max = planeSize.x() / 2;
        //real_T y_min = -planeSize.y() / 2, y_max = planeSize.y() / 2;



        return Pose();
    }

    Vector3r rayToDepthPlaneIntersection(const Vector3r& ray_n, real_T dist)
    {
        // Compute the intersection point on the plane
        real_T x = dist;
        real_T y = dist * ray_n.y() / ray_n.x();
        real_T z = dist * ray_n.z() / ray_n.x();
        // output contact point
        return Vector3r(x, y, z);
    }

};

}}
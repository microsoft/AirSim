#pragma once

#include "DepthNav.hpp"

namespace msr { namespace airlib {

class DepthNavCost : public DepthNav
{
protected:
    virtual Pose getNextPose(const std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt) override
    {
        auto goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);
        if (true) {
            //1. Let's have a plane that fits in our frustum at x = 1 (remember +X is front, +Y is right in NED)
            Vector2r planeSize = getPlaneSize(params_.max_allowed_obs_dist, params_.fov, hfov2vfov(params_.fov, params_.depth_height, params_.depth_width));
            //2. We will compute x_min, y_min, x_max, y_max for this plane in body frame.
            float z_min = -planeSize.x() / 2;
            float y_min = -planeSize.y() / 2;
            float z_max = planeSize.x() / 2;
            float y_max = planeSize.y() / 2;
            //3. Then we will compute x_goal,y_goal where the vector goal_body intersects this plane.
            //ERROR: goal_body should be normalized?
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
                //5. Then descretise the rectangle in M * N cells. This is simply truncation after division. For each pixel we can now get cell coordinates.
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
                float cost;
                float min_cost = FLT_MAX;
                int min_cost_i = 0;
                for (int i = 0; i < cell_centers.size(); ++i) {
                    cost = computeCellCost(depth_image, cell_centers[spiral_idxs[i]], Vector2r(y_px, z_px));
                    if (cost < min_cost) {
                        min_cost = cost;
                        min_cost_i = i;
                    }
                    if (min_cost < 1.0f) break; //decent cost found, move on
                }

                //8. We are here if we have found cell coordinates i, j as center of the free window from step #7
                //9. Compute i_x_center, j_y_center that would be center pixel of this cell in the plane for x = 1
                goal_px = cell_centers[spiral_idxs[min_cost_i]];

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
        //Not in frustrum
        else {
            return Pose::nanPose();
        }
    }

    float computeCellCost(const std::vector<float>& img, Vector2r cell_center, Vector2r goal) {

        unsigned int counter = 0;
        unsigned int count_min_depth = 0;
        float depth_sum = 0;
        Vector2r diff = goal - cell_center;
        float dist_to_goal = sqrt(diff.dot(diff));

        for (int i = int(cell_center.y() - params_.vehicle_height_px / 2); i < int(cell_center.y() + params_.vehicle_height_px / 2); i++) {
            for (int j = int(cell_center.x() - params_.vehicle_width_px / 2); j<int(cell_center.x() + params_.vehicle_width_px / 2); j++) {
                int idx = i * params_.depth_width + j;
                counter++;
                depth_sum += img[idx];
                if (img[idx] < params_.max_allowed_obs_dist) {
                    count_min_depth++;
                }
            }
        }

        return (counter / depth_sum) * (2 ^ count_min_depth) * dist_to_goal;

    }

};

}}
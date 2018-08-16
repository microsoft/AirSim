#pragma once

#include "common/Common.hpp"

namespace msr {
namespace airlib {

class DepthNavOptAStar {
public:
    struct Params {
        //Camera FOV
        real_T hfov = Utils::degreesToRadians(90.0f);

        //depth image dimension, index of pixel x,y = x*width + y 
        unsigned int depth_width = 256, depth_height = 144;

        //flight envelop, center of envelop is center of depth map
        unsigned int env_width = 128, env_height = 72;

        real_T p_opening_per_meter = 0.1f;

        real_T max_obs_dist = 1.0f;

        real_T collision_cost = 1.0E8f;

        unsigned int ray_samples_count = 25;
        unsigned int env_x_oofset, env_y_oofset;
        real_T vfov, aspect;
        real_T tan_hfov_by_2, tan_vfov_by_2;
    };


private:
    struct SampleRay {
        unsigned int pixel_x, pixel_y;
        unsigned int index;
        real_T cost;
        Vector3r ray;
        real_T obs_dist;
    };
    const unsigned int extra_rays = 1;
public:
    DepthNavOptAStar(const Params& params = Params())
        : params_(params),
        sample_rays(params.ray_samples_count + extra_rays), //add two more rays, for origin and goal
        rnd_width_(0, params.env_width - 1), rnd_height_(0, params.env_height - 1)
    {
        params_.aspect = real_T(params_.depth_height) / real_T(params_.depth_width);
        params_.vfov = 2 * std::atan(std::tan(params_.hfov / 2) * params_.aspect);
        params_.env_x_oofset = (params_.depth_width - params_.env_width) / 2;
        params_.env_y_oofset = (params_.depth_height - params_.env_height) / 2;
        params_.tan_hfov_by_2 = std::tan(params_.hfov / 2);
        params_.tan_vfov_by_2 = std::tan(params_.vfov / 2);

        //origin ray
        SampleRay& sample_ray = sample_rays.at(0);
        sample_ray.pixel_x = params_.depth_width / 2;
        sample_ray.pixel_y = params_.depth_height / 2;
    }

    Pose getNextPose(const std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
    {
        Vector3r goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);
        real_T goal_dist = goal_body.norm();

        SampleRay* min_cost_ray = &sample_rays.at(0);
        setupRay(*min_cost_ray, depth_image, goal_dist);

        //sample rays
        for (unsigned int ray_index = 0; ray_index < params_.ray_samples_count; ++ray_index) {
            SampleRay& sample_ray = sample_rays.at(ray_index + extra_rays);
            sample_ray.pixel_x = params_.env_x_oofset + rnd_width_.next();
            sample_ray.pixel_y = params_.env_y_oofset + rnd_height_.next();
            setupRay(sample_ray, depth_image, goal_dist);

            if (min_cost_ray->cost > sample_ray.cost)
                min_cost_ray = &sample_ray;
        }

        return Pose();
    }

private:
    void setupRay(SampleRay& sample_ray, const std::vector<float>& depth_image, real_T goal_dist)
    {
        sample_ray.index = sample_ray.pixel_y * params_.env_width + sample_ray.pixel_x;
        sample_ray.obs_dist = depth_image.at(sample_ray.index);
        sample_ray.ray = pixel2ray(sample_ray.pixel_x, sample_ray.pixel_y);
        sample_ray.cost = computeRayCost(sample_ray, goal_dist);
    }

    Vector3r pixel2ray(unsigned int x, unsigned y)
    {
        real_T pixel_y_n = params_.depth_width / 2.0f - x;
        real_T pixel_z_n = y - params_.depth_height / 2.0f;
        Vector3r ray(1.0f,
            std::tan(pixel_y_n * 2 * params_.tan_hfov_by_2 / params_.depth_width),
            std::tan(pixel_z_n * 2 * params_.tan_vfov_by_2 / params_.depth_height));
        ray.normalize();
        return ray;
    }

    real_T computeRayCost(const SampleRay& sample_ray, real_T goal_dist)
    {
        real_T goal_dist_2 = goal_dist * goal_dist;
        real_T cost = goal_dist;
        real_T turn_dot = sample_ray.ray.dot(VectorMath::front());
        real_T turn_dot_n = (1 - turn_dot) / 2;
        real_T turn_cost = goal_dist_2 * turn_dot_n;

        cost += turn_cost;

        real_T p_no_opening = 1 - std::pow(1 - params_.p_opening_per_meter, sample_ray.obs_dist);
        real_T off_fov_cost = 2 * goal_dist + goal_dist_2;
        real_T backtrack_cost = (cost + off_fov_cost) * p_no_opening;

        cost += backtrack_cost;

        if (sample_ray.obs_dist < params_.max_obs_dist)
            cost += params_.collision_cost;

        return cost;
    }

private:
    Params params_;
    std::vector<SampleRay> sample_rays;
    common_utils::RandomGeneratorUI rnd_width_, rnd_height_;
};

}
}
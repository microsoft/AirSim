#pragma once

#include "common/Common.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/bitmap_image.hpp"
#include "common/common_utils/ColorUtils.hpp"

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
        
        real_T d2_panelty = 6;
        real_T turn_panelty = 10;

        real_T d1_zero_epsilon = 0.1f;

        real_T min_exit_dist_from_goal = 1.0f;
        
        real_T control_loop_period = 30.0f / 1000; //sec
        real_T max_linear_speed = 10; // m/s
        real_T max_angular_speed = 6; // rad/s

        real_T vfov, aspect;
        unsigned int env_x_oofset, env_y_oofset;
        real_T tan_hfov_by_2, tan_vfov_by_2;
    };

    class DepthNavException : public std::runtime_error {
    public:
        DepthNavException(const std::string& message)
            : std::runtime_error(message)
        {}
    };

private:
    struct SampleRay {
        unsigned int pixel_x, pixel_y;
        unsigned int index;
        real_T cost;
        Vector3r ray;
        real_T obs_dist;
        Vector3r d1_v;
        Vector3r d2_v;
        bool has_collision;
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

    virtual void gotoGoal(const Pose& goal_pose, RpcLibClientBase& client)
    {
        typedef ImageCaptureBase::ImageRequest ImageRequest;
        typedef ImageCaptureBase::ImageResponse ImageResponse;
        typedef ImageCaptureBase::ImageType ImageType;

        iteration_index_ = 0;

        do {
            std::vector<ImageRequest> request = {
                ImageRequest("1", ImageType::DepthPlanner, true) /*,
                                                                 ImageRequest("1", ImageType::Scene),
                                                                 ImageRequest("1", ImageType::DisparityNormalized, true) */
            };

            const std::vector<ImageResponse>& response = client.simGetImages(request);

            if (response.size() == 0)
                throw std::length_error("No images received!");

            const Pose current_pose(response.at(0).camera_position, response.at(0).camera_orientation);
            const Pose next_pose = getNextPose(response.at(0).image_data_float, goal_pose.position,
                current_pose, params_.control_loop_period);

            if (VectorMath::hasNan(next_pose))
                throw DepthNavException("No further path can be found.");
            else { //convert pose to velocity commands
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
            }

            real_T dist2goal = getDistanceToGoal(next_pose.position, goal_pose.position);
            if (dist2goal <= params_.min_exit_dist_from_goal)
                return;
            Utils::log(Utils::stringf("Distance to target: %f", dist2goal));

            ++iteration_index_;
        } while (true);
    }

protected:
    Pose getNextPose(const std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
    {
        Vector3r goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);
        real_T goal_dist = goal_body.norm();

        SampleRay* min_cost_ray = &sample_rays.at(0);
        setupRay(*min_cost_ray, depth_image, goal_body, goal_dist);

        if (generate_debug_info_) {
            const auto& bmp = depth2bmp(depth_image);
            writeToBmpFile(bmp, params_.depth_width, params_.depth_height,
                common_utils::FileSystem::combine(std::string("d:\\temp\\111\\"), Utils::stringf("disparity_ % 06d.bmp", iteration_index_)));
        }

        //sample rays
        for (unsigned int ray_index = 0; ray_index < params_.ray_samples_count; ++ray_index) {
            SampleRay& sample_ray = sample_rays.at(ray_index + extra_rays);
            sample_ray.pixel_x = params_.env_x_oofset + rnd_width_.next();
            sample_ray.pixel_y = params_.env_y_oofset + rnd_height_.next();
            setupRay(sample_ray, depth_image, goal_body, goal_dist);

            if (min_cost_ray->cost > sample_ray.cost)
                min_cost_ray = &sample_ray;
        }

        Vector3r next_pos = min_cost_ray->d1_v;
        Quaternionr next_q = min_cost_ray->d1_v.isZero(params_.d1_zero_epsilon) ?
            VectorMath::toQuaternion(VectorMath::front(), min_cost_ray->d2_v.normalized()) :
            VectorMath::toQuaternion(VectorMath::front(), min_cost_ray->d1_v.normalized());

        Pose local_pose(next_pos, next_q);
        Pose global_pose = VectorMath::transformToWorldFrame(local_pose, current_pose, true);
        return global_pose;
    }

    static std::vector<common_utils::bmp::rgb_t> depth2bmp(const std::vector<float>& depth_image)
    {
        std::vector<common_utils::bmp::rgb_t> r(depth_image.size());

        for (unsigned int i = 0; i < depth_image.size(); ++i) {
            float depth = depth_image.at(i);
            float inv_depth = Utils::clip(1 / (depth + 1), 0.0f, 1.0f) / 1.0f;

            //r[i] = common_utils::bmp::convert_wave_length_nm_to_rgb(inv_depth * (725 - 400) + 400);
            common_utils::ColorUtils::valToRGB(inv_depth, r[i].red, r[i].green, r[i].blue);
        }

        return r;
    }

    static void writeToBmpFile(const std::vector<common_utils::bmp::rgb_t>& image, unsigned int width, unsigned int height, const std::string& filepath)
    {
        unsigned int idx = 0;
        common_utils::bmp::bitmap_image img(width, height);
        for (unsigned int j = 0; j < height; ++j) {
            for (unsigned int i = 0; i < width; ++i) {
                img.set_pixel(i, j, image.at(idx));
                ++idx;
            }
        }
        img.save_image(filepath);
    }

    void setupRay(SampleRay& sample_ray, const std::vector<float>& depth_image, const Vector3r& goal_body, real_T goal_dist)
    {
        sample_ray.index = sample_ray.pixel_y * params_.env_width + sample_ray.pixel_x;
        sample_ray.obs_dist = depth_image.at(sample_ray.index);
        sample_ray.ray = pixel2ray(sample_ray.pixel_x, sample_ray.pixel_y);
        setRayCost(sample_ray, goal_body, goal_dist);
    }

    real_T getDistanceToGoal(Vector3r current_position, Vector3r goal)
    {
        Vector3r goalVec = goal - current_position;
        return goalVec.norm();
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

    void setRayCost(SampleRay& sample_ray, const Vector3r& goal_body, real_T goal_dist)
    {
        real_T goal_on_ray = std::min(goal_body.dot(sample_ray.ray), 0.0f);
        real_T d1 = std::min(sample_ray.obs_dist, goal_on_ray);

        sample_ray.d1_v = sample_ray.ray * d1;
        sample_ray.d2_v = goal_body - sample_ray.d1_v;
        real_T d2 = sample_ray.d2_v.norm();
        real_T d2_penalized = d2 * params_.d2_panelty;

        real_T turn_dot1 = (1 - sample_ray.ray.dot(VectorMath::front())) / 2;
        real_T turn_dot2 = (1 - sample_ray.ray.dot(sample_ray.d2_v)) / 2;

        sample_ray.cost = d1 + d2_penalized + params_.turn_panelty * (turn_dot1 + turn_dot2);

        if (sample_ray.obs_dist < params_.max_obs_dist) {
            sample_ray.has_collision = true; 
            sample_ray.cost += 1.0E15f;
        }
    }

private:
    Params params_;
    std::vector<SampleRay> sample_rays;
    common_utils::RandomGeneratorUI rnd_width_, rnd_height_;
    bool generate_debug_info_ = true;
    unsigned int iteration_index_;
};

}
}
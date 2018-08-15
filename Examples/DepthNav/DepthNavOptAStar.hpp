#pragma once

#include "common/Common.hpp"

namespace msr { namespace airlib {

class DepthNavOptAStar  {
public:
    struct Params {
        //Camera FOV
        real_T hfov = Utils::degreesToRadians(90.0f);

        //depth image dimension, index of pixel x,y = x*width + y 
        unsigned int depth_width = 256, depth_height = 144;
        //flight envelop width, height
        real_T env_width = 0.8f, env_height = 0.8f;

        unsigned int ray_samples_count = 25;

        real_T vfov, aspect;

        void recompute()
        {
            aspect = real_T(depth_height) / real_T(depth_width);
            vfov = 2 * std::atan(std::tan(hfov / 2) * aspect);
        }

    private:
        void hfov2vfov()
        {
        }
    };

public:
    DepthNavOptAStar(const Params& params = Params())
        : params_(params), sample_rays(params.ray_samples_count),
            rnd_width_(0, params.depth_width-1)
    {
    }

    Pose getNextPose(const std::vector<float>& depth_image, const Vector3r& goal, const Pose& current_pose, real_T dt)
    {

        Vector3r goal_body = VectorMath::transformToBodyFrame(goal, current_pose, true);

        

        return Pose();
    }

private:
    struct SampleRay {

    };

private:
    const Params params_;
    std::vector<SampleRay> sample_rays;
    common_utils::RandomGeneratorI rnd_width_, rnd_height_;
};

}}
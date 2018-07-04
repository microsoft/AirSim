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
        float fov = Utils::degreesToRadians(90.0f);
        //min dot product required for goal vector to be
        //considered so goal is inside frustum
        float min_frustrum_alighment = (1.0f + 0.75f) / 2;
    };

public:
    DepthNav(const Params& params = Params())
        : params_(params)
    {
    }

    //goal is specified in world frame and typically provided by the global planner
    //current_pose is current pose of the vehicle in world frame
    //return next pose that vehicle should be in
    Pose getNextPose(const Vector3r& goal, const Pose& current_pose)
    {
        //get goal in body frame
        auto goal_body = VectorMath::transformToBodyFrame(goal, current_pose);

        //is goal inside frustum?
        float frustrum_alighment = std::fabsf(goal_body.dot(VectorMath::front())); 
        if (frustrum_alighment >= params_.min_frustrum_alighment) {

        }
        else {
            //goal is not in the frustum. Let's rotate ourselves until we get 
            //goal in the frustum

        }
    }

private:
    Params params_;
};

}}
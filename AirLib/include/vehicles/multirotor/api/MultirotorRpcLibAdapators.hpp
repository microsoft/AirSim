// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibAdapators_hpp
#define air_MultirotorRpcLibAdapators_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "vehicles/multirotor/controllers/DroneCommon.hpp"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "controllers/ImageCaptureBase.hpp"
#include "safety/SafetyEval.hpp"
#include "rpc/msgpack.hpp"


namespace msr { namespace airlib_rpclib {

class MultirotorRpcLibAdapators : public RpcLibAdapatorsBase {
public:
    struct YawMode {
        bool is_rate;
        float yaw_or_rate;
        MSGPACK_DEFINE_MAP(is_rate, yaw_or_rate);
    
        YawMode()
        {}

        YawMode(const msr::airlib::YawMode& s)
        {
            is_rate = s.is_rate;
            yaw_or_rate = s.yaw_or_rate;
        }
        msr::airlib::YawMode to() const
        {
            return msr::airlib::YawMode(is_rate, yaw_or_rate);
        }
    };

};

}} //namespace

MSGPACK_ADD_ENUM(msr::airlib::DrivetrainType);


#endif

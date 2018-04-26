// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/car/api/CarRpcLibServer.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
//#undef check
#include "rpc/server.h"
#include "vehicles/car/api/CarRpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON


namespace msr { namespace airlib {

typedef msr::airlib_rpclib::CarRpcLibAdapators CarRpcLibAdapators;

CarRpcLibServer::CarRpcLibServer(SimModeApiBase* simmode_api, string server_address, uint16_t port)
    : RpcLibServerBase(simmode_api, server_address, port)
{
    (static_cast<rpc::server*>(getServer()))->
        bind("getCarState", [&]() -> CarRpcLibAdapators::CarState {
        return CarRpcLibAdapators::CarState(getCarApi()->getCarState());
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("setCarControls", [&](const CarRpcLibAdapators::CarControls& controls) -> void {
        getCarApi()->setCarControls(controls.to());
    });

}

//required for pimpl
CarRpcLibServer::~CarRpcLibServer()
{
}

CarApiBase* CarRpcLibServer::getCarApi() const
{
    return static_cast<CarApiBase*>(getSimModeApi()->getVehicleApi());
}


}} //namespace


#endif
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibDroneServer_hpp
#define air_RpcLibDroneServer_hpp

#include "controllers/DroneControllerCancelable.hpp"
#include "RpcLibBaseServer.hpp"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#undef check
#include "rpc/server.h"
#include "api/RpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON

namespace msr { namespace airlib {

class RpcLibDroneServer : public RpcLibBaseServer {
public:
    RpcLibDroneServer(DroneControllerCancelable* drone, string server_address, uint16_t port = 41451);
    virtual ~RpcLibDroneServer() override;
private:
    DroneControllerCancelable* drone_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibBaseServer_hpp
#define air_RpcLibBaseServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "ControlServerBase.hpp"

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

class RpcLibBaseServer : public ControlServerBase {

protected:
    typedef msr::airlib_rpclib::RpcLibAdapators RpcLibAdapators;

    struct impl {
        impl(string server_address, uint16_t port)
            : server(server_address, port)
        {}

        impl(uint16_t port)
            : server(port)
        {}

        ~impl() {
        }

        rpc::server server;
    };
    std::unique_ptr<impl> pimpl_;
public:
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~RpcLibBaseServer() override;

};

}} //namespace
#endif

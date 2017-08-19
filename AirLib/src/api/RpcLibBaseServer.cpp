// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "api/RpcLibBaseServer.hpp"

namespace msr { namespace airlib {


//required for pimpl
RpcLibBaseServer::~RpcLibBaseServer()
{
    stop();
}

void RpcLibBaseServer::start(bool block)
{
    if (block)
        pimpl_->server.run();
    else
        pimpl_->server.async_run(4);   //4 threads
}

void RpcLibBaseServer::stop()
{
    pimpl_->server.stop();
}

}} //namespace


#endif
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServer_hpp
#define air_RpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "DroneControlServer.hpp"
#include "ControlServerBase.hpp"

namespace msr { namespace airlib {

class RpcLibServer : ControlServerBase {
public:
    RpcLibServer(DroneControlServer* drone, string server_address, uint16_t port = 41451);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~RpcLibServer() override;
private:
    DroneControlServer* drone_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

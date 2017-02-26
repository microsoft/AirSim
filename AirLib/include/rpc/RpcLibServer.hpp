// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServer_hpp
#define air_RpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "controllers/DroneControllerCancelable.hpp"
#include "ControlServerBase.hpp"

namespace msr { namespace airlib {

class RpcLibServer : ControlServerBase {
public:
    RpcLibServer(DroneControllerCancelable* drone, string server_address, uint16_t port = 41451);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~RpcLibServer() override;
private:
    DroneControllerCancelable* drone_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

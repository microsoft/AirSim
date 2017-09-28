// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibServer_hpp
#define air_MultirotorRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "vehicles/multirotor/controllers/DroneControllerCancelable.hpp"
#include "api/ControlServerBase.hpp"

namespace msr { namespace airlib {

class MultirotorRpcLibServer : public ControlServerBase {
public:
    MultirotorRpcLibServer(DroneControllerCancelable* drone, string server_address, uint16_t port = 41451);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~MultirotorRpcLibServer() override;
private:
    DroneControllerCancelable* drone_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

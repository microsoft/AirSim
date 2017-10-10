// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibServer_hpp
#define air_CarRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/ControlServerBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

namespace msr { namespace airlib {

class CarRpcLibServer : public ControlServerBase {
public:
    CarRpcLibServer(CarApiBase* vehicle, string server_address, uint16_t port = 42451);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~CarRpcLibServer() override;

private:
    CarApiBase* vehicle_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif
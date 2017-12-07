// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServerBase_hpp
#define air_RpcLibServerBase_hpp

#include "common/Common.hpp"
#include "api/ControlServerBase.hpp"
#include "api/VehicleApiBase.hpp"


namespace msr { namespace airlib {


class RpcLibServerBase : public ControlServerBase {
public:
    RpcLibServerBase(VehicleApiBase* vehicle, string server_address, uint16_t port);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~RpcLibServerBase() override;

protected:
    void* getServer();
    VehicleApiBase* getVehicleApi();

private:
    VehicleApiBase* vehicle_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};


}} //namespace
#endif
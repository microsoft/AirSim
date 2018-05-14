// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServerBase_hpp
#define air_RpcLibServerBase_hpp

#include "common/Common.hpp"
#include "api/ApiServerBase.hpp"
#include "api/SimModeApiBase.hpp"


namespace msr { namespace airlib {


class RpcLibServerBase : public ApiServerBase {
public:
    RpcLibServerBase(SimModeApiBase* simmode_api, string server_address, uint16_t port);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~RpcLibServerBase() override;

protected:
    void* getServer() const;
    SimModeApiBase* getSimModeApi() const;

private:
    VehicleApiBase* getVehicleApi() const;

private:
    SimModeApiBase* simmode_api_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};


}} //namespace
#endif
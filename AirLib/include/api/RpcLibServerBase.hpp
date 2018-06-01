// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServerBase_hpp
#define air_RpcLibServerBase_hpp

#include "common/Common.hpp"
#include "api/ApiServerBase.hpp"
#include "api/ApiProvider.hpp"


namespace msr { namespace airlib {


class RpcLibServerBase : public ApiServerBase {
public:
    RpcLibServerBase(ApiProvider* api_provider, const std::string& server_address, uint16_t port);
    virtual ~RpcLibServerBase() override;

    virtual void start(bool block = false) override;
    virtual void stop() override;

protected:
    void* getServer() const;


    virtual VehicleApiBase* getVehicleApi(const std::string& vehicle_name = "")
    {
        return api_provider_->getVehicleApi(vehicle_name);
    }
    virtual VehicleSimApiBase* getVehicleSimApi(const std::string& vehicle_name = "")
    {
        return api_provider_->getVehicleSimApi(vehicle_name);
    }
    virtual WorldSimApiBase* getWorldSimApi()
    {
        return api_provider_->getWorldSimApi();
    }


private:
    ApiProvider* api_provider_;

    struct impl;
    std::unique_ptr<impl> pimpl_;
};


}} //namespace
#endif
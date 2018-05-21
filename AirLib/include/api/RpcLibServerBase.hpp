// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibServerBase_hpp
#define air_RpcLibServerBase_hpp

#include "common/Common.hpp"
#include "api/ApiServerBase.hpp"
#include "api/WorldSimApiBase.hpp"


namespace msr { namespace airlib {


class RpcLibServerBase : public ApiServerBase {
public:
    RpcLibServerBase(const std::string& server_address, uint16_t port);
    virtual ~RpcLibServerBase() override;

    virtual void start(bool block = false) override;
    virtual void stop() override;

protected:
    void* getServer() const;

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};


}} //namespace
#endif
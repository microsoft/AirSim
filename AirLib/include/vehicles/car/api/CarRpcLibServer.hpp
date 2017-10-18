// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibServer_hpp
#define air_CarRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/car/api/CarApiBase.hpp"

namespace msr { namespace airlib {

class CarRpcLibServer : public RpcLibServerBase {
public:
    CarRpcLibServer(CarApiBase* vehicle, string server_address, uint16_t port = 42451);
    virtual ~CarRpcLibServer();

private:
    CarApiBase* getCarApi();
};

}} //namespace
#endif
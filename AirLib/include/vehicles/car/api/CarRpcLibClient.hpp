// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarRpcLibClient_hpp
#define air_CarRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "common/ImageCaptureBase.hpp"


namespace msr { namespace airlib {

class CarRpcLibClient : public RpcLibClientBase {
public:
    CarRpcLibClient(const string& ip_address = "localhost", uint16_t port = 42451, uint timeout_ms = 60000);
    void setCarControls(const CarApiBase::CarControls& controls);
    void reset();
    CarApiBase::CarState getCarState();

    virtual ~CarRpcLibClient();    //required for pimpl
};

}} //namespace
#endif

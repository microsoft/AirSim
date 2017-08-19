// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibAutomobileServer_hpp
#define air_RpcLibAutomobileServer_hpp

#include "controllers/AutomobileSedanController.hpp"
#include "RpcLibBaseServer.hpp"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#undef check
#include "rpc/server.h"
#include "api/RpcLibAdapators.hpp"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#define check(expr) (static_cast<void>((expr)))
STRICT_MODE_ON

namespace msr { namespace airlib {

class RpcLibAutomobileServer : RpcLibBaseServer {
public:
    RpcLibAutomobileServer(AutomobileSedanController* vechicle_controller, string server_address, uint16_t port = 41451);
    virtual ~RpcLibAutomobileServer() override;

protected:
    bool setVehicleControlSignals(float steeringAngle, float throttlePercentage, float brakePercentage);
    vector<uint8_t> getImageFromCamera(uint8_t camera_id, VehicleCameraBase::ImageType_ type);

private:
    AutomobileSedanController* vehicle_controller_;
};

}} //namespace
#endif

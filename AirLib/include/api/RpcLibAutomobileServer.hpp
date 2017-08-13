// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RpcLibAutomobileServer_hpp
#define air_RpcLibAutomobileServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "controllers/AutomobileSedanController.hpp"
#include "ControlServerBase.hpp"

namespace msr { namespace airlib {

class RpcLibAutomobileServer : ControlServerBase {
public:
    RpcLibAutomobileServer(AutomobileSedanController* vechicle_controller, string server_address, uint16_t port = 41451);
    virtual void start(bool block = false) override;
    virtual void stop() override;
    virtual ~RpcLibAutomobileServer() override;

protected:
    bool setVehicleControlSignals(float steeringAngle, float throttlePercentage, float brakePercentage);
	vector<uint8_t> getImageFromCamera(uint8_t camera_id, VehicleCameraBase::ImageType_ type);

private:
    AutomobileSedanController* vehicle_controller_;
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif

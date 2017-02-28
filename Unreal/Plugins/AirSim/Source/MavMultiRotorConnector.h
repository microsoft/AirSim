#pragma once

#include "controllers/MavLinkDroneController.hpp"
#include "controllers/DroneControllerCancelable.hpp"
#include "rpc/RpcLibServer.hpp"
#include "vehicles/configs/Px4QuadX.hpp"
#include "vehicles/MultiRotor.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "VehicleConnectorBase.h"
#include "FlyingPawn.h"
#include <chrono>

class MavMultiRotorConnector : public VehicleConnectorBase
{
public:
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::ControllerBase ControllerBase;
    typedef msr::airlib::MultiRotor MultiRotor;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
        
public:
	virtual ~MavMultiRotorConnector();

    //VehicleConnectorBase interface
    //implements game interface to update pawn
    void initialize(AFlyingPawn* vehicle_pawn);
    virtual void beginPlay() override;
    virtual void endPlay() override;
    virtual void updateRenderedState() override;
    virtual void updateRendering(float dt) override;

    virtual void startApiServer() override;
    virtual void stopApiServer() override;
    virtual bool isApiServerStarted() override;
    virtual msr::airlib::VehicleControllerBase* getController() override;

    //PhysicsBody interface
    //this just wrapped around MultiRotor physics body
    virtual void reset() override;
    virtual void update(real_T dt) override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

private:
    msr::airlib::MavLinkDroneController::ConnectionInfo getConnectionInfo();
    void createController(MultiRotor& vehicle);

private:
    MultiRotor vehicle_;
    msr::airlib::Px4QuadX vehicle_params_;
    std::unique_ptr<msr::airlib::DroneControllerBase> controller_;
    std::vector<std::string> controller_messages_;
    msr::airlib::Environment environment_;
    AFlyingPawn* vehicle_pawn_;

    std::unique_ptr<msr::airlib::DroneControllerCancelable> controller_cancelable_;
    std::unique_ptr<msr::airlib::RpcLibServer> rpclib_server_;

    real_T rotor_speeds_[4];
    int rotor_directions_[4];
    real_T rotor_thrusts_[4];
    real_T rotor_controls_filtered_[4];
};

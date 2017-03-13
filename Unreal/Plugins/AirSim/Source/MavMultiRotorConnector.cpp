#include "AirSim.h"
#include "MavMultiRotorConnector.h"
#include "vehicles/configs/PX4QuadX.hpp"
#include "AirBlueprintLib.h"

using namespace msr::airlib;

void MavMultiRotorConnector::initialize(AFlyingPawn* vehicle_pawn)
{
    vehicle_params_.reset(new msr::airlib::Px4QuadX(vehicle_pawn->getVehicleName()));
    vehicle_params_->initialize();

	vehicle_pawn_ = vehicle_pawn;
	vehicle_pawn_->initialize();

	//init physics vehicle
	auto initial_kinematics = Kinematics::State::zero();
	initial_kinematics.pose = vehicle_pawn_->getPose();
	msr::airlib::Environment::State initial_environment;
	initial_environment.position = initial_kinematics.pose.position;
	initial_environment.geo_point = vehicle_pawn_->getHomePoint();
	initial_environment.min_z_over_ground = vehicle_pawn_->getMinZOverGround();
	environment_.initialize(initial_environment);
	vehicle_.initialize(vehicle_params_.get(), initial_kinematics,
		&environment_);
}

MavMultiRotorConnector::~MavMultiRotorConnector()
{
	stopApiServer();
}

void MavMultiRotorConnector::beginPlay()
{
    //connect to HIL
    try {

        vehicle_.getController()->start();
    }
    catch (std::exception ex) {

        UAirBlueprintLib::LogMessage(FString("Vehicle controller cannot be started, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 180);
    }
}

msr::airlib::VehicleControllerBase* MavMultiRotorConnector::getController()
{
    return vehicle_.getController();
}

void MavMultiRotorConnector::endPlay()
{
    vehicle_.getController()->stop();
}

void MavMultiRotorConnector::updateRenderedState()
{
	//move collison info from rendering engine to vehicle
	vehicle_.setCollisionInfo(vehicle_pawn_->getCollisonInfo());
	//update ground level
	environment_.getState().min_z_over_ground = vehicle_pawn_->getMinZOverGround();
	//update pose of object in rendering engine
	vehicle_pawn_->setPose(vehicle_.getPose());

	//update rotor poses
	for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
		const auto& rotor_output = vehicle_.getRotorOutput(i);
		rotor_speeds_[i] = rotor_output.speed;
		rotor_directions_[i] = static_cast<int>(rotor_output.turning_direction);
		rotor_thrusts_[i] = rotor_output.thrust;
		rotor_controls_filtered_[i] = rotor_output.control_signal_filtered;
	}

    vehicle_.getController()->getStatusMessages(controller_messages_);
}

void MavMultiRotorConnector::updateRendering(float dt)
{
	try {
		vehicle_.getController()->reportTelemetry(dt);
	}
	catch (std::exception &e) {
		UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
	}

	//update rotor animations
	for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
		vehicle_pawn_->setRotorSpeed(i, rotor_speeds_[i] * rotor_directions_[i]);
	}

	for (auto i = 0; i < controller_messages_.size(); ++i) {
		UAirBlueprintLib::LogMessage(FString(controller_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
	}
}


void MavMultiRotorConnector::startApiServer()
{
    //TODO: remove static up cast from below?
    controller_cancelable_.reset(new msr::airlib::DroneControllerCancelable(
        vehicle_.getController()));
    std::string server_address = Settings::singleton().getString("LocalHostIp", "127.0.0.1");
    rpclib_server_.reset(new msr::airlib::RpcLibServer(controller_cancelable_.get(), server_address));
    rpclib_server_->start();

}
void MavMultiRotorConnector::stopApiServer()
{
	if (rpclib_server_ != nullptr) {
		rpclib_server_->stop();
		rpclib_server_.reset(nullptr);
		controller_cancelable_.reset(nullptr);
	}
}

bool MavMultiRotorConnector::isApiServerStarted()
{
    return rpclib_server_ != nullptr;
}

//*** Start: UpdatableState implementation ***//
void MavMultiRotorConnector::reset()
{
	vehicle_pawn_->reset();    //we do flier resetPose so that flier is placed back without collisons
	vehicle_.reset();
}

void MavMultiRotorConnector::update(real_T dt)
{
	//this is high frequency physics tick, flier gets ticked at rendering frame rate
	vehicle_.update(dt);
}

void MavMultiRotorConnector::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = vehicle_pawn_->getPosition();
    reporter.writeValue("unreal pos", AVehiclePawnBase::toVector3r(unrealPosition, 1.0f, false));

	vehicle_.reportState(reporter);

}

MavMultiRotorConnector::UpdatableObject* MavMultiRotorConnector::getPhysicsBody()
{
	return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//


#include "AirSim.h"
#include "MultiRotorConnector.h"
#include "AirBlueprintLib.h"
#include "controllers/Settings.hpp"
#include <exception>

using namespace msr::airlib;

void MultiRotorConnector::initialize(AFlyingPawn* vehicle_pawn, msr::airlib::MultiRotorParams* vehicle_params)
{
    vehicle_pawn_ = vehicle_pawn;
    vehicle_pawn_->initialize();

    vehicle_params_ = vehicle_params;

    //init physics vehicle
    auto initial_kinematics = Kinematics::State::zero();
    initial_kinematics.pose = vehicle_pawn_->getPose();
    msr::airlib::Environment::State initial_environment;
    initial_environment.position = initial_kinematics.pose.position;
    initial_environment.geo_point = vehicle_pawn_->getHomePoint();
    initial_environment.min_z_over_ground = vehicle_pawn_->getMinZOverGround();
    environment_.initialize(initial_environment);

    vehicle_params_->initialize();
    vehicle_.initialize(vehicle_params_, initial_kinematics, &environment_);

    //pass ground truth to some controllers who needs it (usually the ones without state estimation capabilities)
    vehicle_params_->initializePhysics(&environment_, &vehicle_.getKinematics());
}

MultiRotorConnector::~MultiRotorConnector()
{
    stopApiServer();
}

void MultiRotorConnector::beginPlay()
{
    last_pose = Pose::nanPose();
    last_debug_pose = Pose::nanPose();

    //connect to HIL
    try {

        vehicle_.getController()->start();
    }
    catch (std::exception ex) {

        UAirBlueprintLib::LogMessage(FString("Vehicle controller cannot be started, please check your settings.json"), FString(ex.what()), LogDebugLevel::Failure, 180);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 180);
    }
}

msr::airlib::VehicleControllerBase* MultiRotorConnector::getController()
{
    return vehicle_.getController();
}

void MultiRotorConnector::endPlay()
{
    vehicle_.getController()->stop();
}

void MultiRotorConnector::updateRenderedState()
{
    //move collison info from rendering engine to vehicle
    vehicle_.setCollisionInfo(vehicle_pawn_->getCollisonInfo());
    //update ground level
    environment_.getState().min_z_over_ground = vehicle_pawn_->getMinZOverGround();
    //update pose of object for rendering engine
    last_pose = vehicle_.getPose();
    last_debug_pose = vehicle_.getController()->getDebugPose();

    //update rotor poses
    for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
        const auto& rotor_output = vehicle_.getRotorOutput(i);
        rotor_speeds_[i] = rotor_output.speed;
        rotor_directions_[i] = static_cast<int>(rotor_output.turning_direction);
        rotor_thrusts_[i] = rotor_output.thrust;
        rotor_controls_filtered_[i] = rotor_output.control_signal_filtered;
    }

    vehicle_.getController()->getStatusMessages(controller_messages_);

    vehicle_.getController()->setRCData(vehicle_pawn_->getRCData());
}

void MultiRotorConnector::updateRendering(float dt)
{
	try {
		vehicle_.getController()->reportTelemetry(dt);
	}
	catch (std::exception &e) {
		UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
	}

    if (!VectorMath::hasNan(last_pose.position)) {
        vehicle_pawn_->setPose(last_pose, last_debug_pose);
    }
    
    //update rotor animations
    for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
        vehicle_pawn_->setRotorSpeed(i, rotor_speeds_[i] * rotor_directions_[i]);
    }

    for (auto i = 0; i < controller_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(controller_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }
}


void MultiRotorConnector::startApiServer()
{
	auto settings = Settings::singleton();
	bool rpcEnabled = settings.getBool("RpcEnabled", false);
	if (settings.setBool("RpcEnabled", rpcEnabled) && settings.isLoadSuccess()) {
		// update settings file so it knows about this new boolean switch
		settings.saveJSonFile(settings.getFileName()); 
	}

	if (rpcEnabled) {
		controller_cancelable_.reset(new msr::airlib::DroneControllerCancelable(
			vehicle_.getController()));
		api_server_address_ = Settings::singleton().getString("LocalHostIp", "127.0.0.1");
		rpclib_server_.reset(new msr::airlib::RpcLibServer(controller_cancelable_.get(), api_server_address_));
		rpclib_server_->start();
	}

}
void MultiRotorConnector::stopApiServer()
{
    if (rpclib_server_ != nullptr) {
        controller_cancelable_->cancelAllTasks();
        rpclib_server_->stop();
        rpclib_server_.reset(nullptr);
        controller_cancelable_.reset(nullptr);
    }
}

bool MultiRotorConnector::isApiServerStarted()
{
    return rpclib_server_ != nullptr;
}

//*** Start: UpdatableState implementation ***//
void MultiRotorConnector::reset()
{
    vehicle_pawn_->reset();    //we do flier resetPose so that flier is placed back without collisons
    vehicle_.reset();
}

void MultiRotorConnector::update(real_T dt)
{
    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    vehicle_.update(dt);
}

void MultiRotorConnector::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    if (vehicle_pawn_ != nullptr) {
        FVector unrealPosition = vehicle_pawn_->getPosition();
        reporter.writeValue("unreal pos", AVehiclePawnBase::toVector3r(unrealPosition, 1.0f, false));
        vehicle_.reportState(reporter);
    }
}

MultiRotorConnector::UpdatableObject* MultiRotorConnector::getPhysicsBody()
{
    return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//


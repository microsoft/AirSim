#include "MultirotorPawnSimApi.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "../../UnitySensors/UnitySensorFactory.h"
#include "../../PInvokeWrapper.h"
#include "../../UnityUtilities.hpp"

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(const Params& params) : PawnSimApi(params)
{
	//reset roll & pitch of vehicle as multirotors required to be on plain surface at start
	Pose pose = getPose();
	float pitch, roll, yaw;
	VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
	pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
	setPose(pose, false);
}

void MultirotorPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnitySensorFactory> sensor_factory = std::make_shared<UnitySensorFactory>(getVehicleName(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();
    //setup physics vehicle
    multirotor_physics_body_ = std::unique_ptr<MultiRotor>(new MultiRotorPhysicsBody(vehicle_params_.get(), vehicle_api_.get(),
        getKinematics(), getEnvironment()));
    rotor_count_ = multirotor_physics_body_->wrenchVertexCount();
    rotor_actuator_info_.assign(rotor_count_, RotorActuatorInfo());

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = pending_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
}

void MultirotorPawnSimApi::updateRenderedState(float dt)
{
	//if reset is pending then do it first, no need to do other things until next tick
	if (reset_pending_)
	{
		reset_task_();
		did_reset_ = true;
		return;
	}

	//move collision info from rendering engine to vehicle
	const CollisionInfo& collision_info = getCollisionInfo();
	multirotor_physics_body_->setCollisionInfo(collision_info);

	if (pending_pose_status_ == PendingPoseStatus::RenderStatePending)
		multirotor_physics_body_->setPose(pending_phys_pose_);

	last_phys_pose_ = multirotor_physics_body_->getPose();

	collision_response = multirotor_physics_body_->getCollisionResponseInfo();

	//update rotor poses
	for (unsigned int i = 0; i < rotor_count_; ++i)
	{
		const auto& rotor_output = multirotor_physics_body_->getRotorOutput(i);
		RotorActuatorInfo* info = &rotor_actuator_info_[i];
		info->rotor_speed = rotor_output.speed;
		info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
		info->rotor_thrust = rotor_output.thrust;
		info->rotor_control_filtered = rotor_output.control_signal_filtered;
	}

	vehicle_api_->getStatusMessages(vehicle_api_messages_);

	if (getRemoteControlID() >= 0)
		vehicle_api_->setRCData(getRCData());
}

void MultirotorPawnSimApi::updateRendering(float dt)
{
	//if we did reset then don't worry about synchronizing states for this tick
	if (reset_pending_)
	{
		// Continue to wait for reset
		if (!did_reset_)
		{
			return;
		}
		else
		{
			reset_pending_ = false;
			did_reset_ = false;
			return;
		}
	}

	if (!VectorMath::hasNan(last_phys_pose_))
	{
		if (pending_pose_status_ == PendingPoseStatus::RenderPending)
		{
			PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
			pending_pose_status_ = PendingPoseStatus::NonePending;
		}
		else
		{
			PawnSimApi::setPose(last_phys_pose_, false);
		}
	}

	PrintLogMessage("Collision Count:", std::to_string(collision_response.collision_count_non_resting).c_str(), getVehicleName().c_str(), ErrorLogSeverity::Information);

	for (auto i = 0; i < vehicle_api_messages_.size(); ++i)
	{
		PrintLogMessage(vehicle_api_messages_[i].c_str(), "30", getVehicleName().c_str(), ErrorLogSeverity::Information);
	}

	try
	{
		vehicle_api_->sendTelemetry(dt);
	}
	catch (std::exception &e)
	{
		PrintLogMessage(e.what(), "LogDebugLevel::Failure, 30", getVehicleName().c_str(), ErrorLogSeverity::Error);
	}

	for (int i = 0; i < rotor_actuator_info_.size(); i++)
	{
		SetRotorSpeed(i, UnityUtilities::Convert_to_UnityRotorInfo(rotor_actuator_info_[i]), getVehicleName().c_str());
	}
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
	pending_phys_pose_ = pose;
	pending_pose_collisions_ = ignore_collision;
	pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::resetImplementation()
{
	PawnSimApi::resetImplementation();

	vehicle_api_->reset();
	multirotor_physics_body_->reset();
	vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void MultirotorPawnSimApi::update()
{
	//environment update for current position
	PawnSimApi::update();

	//update forces on vertices
	multirotor_physics_body_->update();

	//update to controller must be done after kinematics have been updated by physics engine
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
	//// report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
	//AirSimPose pose = GetPose(getVehicleName().c_str());
	//reporter.writeValue("unreal pos", Vector3r(pose.position.x, pose.position.y, pose.position.z));
	multirotor_physics_body_->reportState(reporter);
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
	return multirotor_physics_body_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//


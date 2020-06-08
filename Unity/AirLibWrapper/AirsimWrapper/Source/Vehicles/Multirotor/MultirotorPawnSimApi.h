#pragma once

#include <future>
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "physics//Kinematics.hpp"
#include "MultirotorPawnEvents.h"
#include "../../PawnSimApi.h"

class MultirotorPawnSimApi : public PawnSimApi
{
public:
	typedef msr::airlib::real_T real_T;
	typedef msr::airlib::Utils Utils;
	typedef msr::airlib::MultiRotorPhysicsBody MultiRotor;
	typedef msr::airlib::StateReporter StateReporter;
	typedef msr::airlib::UpdatableObject UpdatableObject;
	typedef msr::airlib::Pose Pose;
	typedef MultirotorPawnEvents::RotorActuatorInfo RotorActuatorInfo;

public:
    virtual void initialize() override;

	MultirotorPawnSimApi(const Params& params);
	virtual ~MultirotorPawnSimApi() = default;
	virtual void updateRenderedState(float dt) override;
	virtual void updateRendering(float dt) override;
	virtual void resetImplementation() override;
	virtual void update() override;
	virtual void reportState(StateReporter& reporter) override;
	virtual UpdatableObject* getPhysicsBody() override;
	virtual void setPose(const Pose& pose, bool ignore_collision) override;

	msr::airlib::MultirotorApiBase* getVehicleApi() const
	{
		return vehicle_api_.get();
	}

private:
	std::unique_ptr<msr::airlib::MultirotorApiBase> vehicle_api_;
	std::unique_ptr<msr::airlib::MultiRotorParams> vehicle_params_;
	std::unique_ptr<MultiRotor> multirotor_physics_body_;
	unsigned int rotor_count_;
	std::vector<RotorActuatorInfo> rotor_actuator_info_;
	enum class PendingPoseStatus
	{
		NonePending,
		RenderStatePending,
		RenderPending
	} pending_pose_status_;

	//show info on collision response from physics engine
	CollisionResponse collision_response;

	//when pose needs to set from non-physics thread, we set it as pending
	bool pending_pose_collisions_;

	Pose pending_phys_pose_; //force new pose through API

	//reset must happen while World is locked so its async task initiated from API thread
	bool reset_pending_;
	bool did_reset_;
	std::packaged_task<void()> reset_task_;
	Pose last_phys_pose_; //for trace lines showing vehicle path
	std::vector<std::string> vehicle_api_messages_;
};
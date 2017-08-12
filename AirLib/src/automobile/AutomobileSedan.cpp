#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif
#include "automobile/AutomobileSedan.hpp"

namespace msr { namespace airlib {

	AutomobileSedan::AutomobileSedan(const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment, std::vector<PneumaticWheel*> wheels, AutomobileEngine* engine, AutomobileBrake* brake, AutomobileStaticParams staticParameters) :
		Automobile(inertia, initial_kinematic_state, environment, wheels, engine, brake, staticParameters)
	{
	}

	void AutomobileSedan::SetControlSignals(real_T steeringAngle, real_T throttlePercentage, real_T brakePercentage)
	{
		this->_automobileState.SteeringAngle = steeringAngle;
		this->_automobileState.BrakePercentage = brakePercentage;
		this->_automobileState.ThrottlePercentage = std::max(static_cast<real_T>(-0.2), throttlePercentage);
	}

	real_T AutomobileSedan::GetEngineTorque(real_T wheelAngularRadiansPerSecond) const
	{
		return this->_engine->GetTorque(this->_automobileState.ThrottlePercentage, wheelAngularRadiansPerSecond);
	}

	real_T AutomobileSedan::GetBrakingTorque() const
	{
		return this->_brake->GetBrakeTorque(this->_automobileState.BrakePercentage);
	}

	real_T AutomobileSedan::GetSteeringAngleInRadians() const 
	{
		return static_cast<real_T>(this->_automobileState.SteeringAngle);
	}

	real_T AutomobileSedan::GetLongitudinalWheelBase() const
	{
		return this->_staticParameters.LongitudinalWheelBase;
	}

	real_T AutomobileSedan::GetLateralWheelBase() const
	{
		return this->_staticParameters.LateralWheelBase;
	}
}}

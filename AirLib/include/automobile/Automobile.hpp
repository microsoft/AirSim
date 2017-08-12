#ifndef air_automobile_hpp
#define air_automobile_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/common_utils/Utils.hpp"
#include "automobile/AutomobileState.hpp"
#include "automobile/AutomobileStaticParams.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/common_utils/Utils.hpp"
#include "automobile/PneumaticWheel.hpp"
#include "automobile/AutomobileEngine.hpp"
#include "automobile/AutomobileBrake.hpp"

#include <vector>

namespace msr { namespace airlib {

	class PneumaticWheel;

	class Automobile : public PhysicsBody {
		
		public:
			Automobile(const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment, std::vector<PneumaticWheel*> wheels, AutomobileEngine* engine, AutomobileBrake* brake, AutomobileStaticParams staticParams) :
				PhysicsBody(staticParams.Mass, inertia, initial_kinematic_state, environment) 
			{
				this->_wheels = wheels;
				this->_staticParameters = staticParams;
				this->_engine = engine;
				this->_brake = brake;

				for (unsigned int i = 0; i < wheels.size(); ++i)
				{
					wheels[i]->AttachAutomobile(this);
				}
			};

			/*
				SteeringAngle is on range [-1, 1]
				ThrottlePercentage is on range [-1, 1]
				BrakePercentage is on range [0, 1]

				Each particular automobile might interpret these parameters differently
			*/
			virtual void SetControlSignals(real_T steeringAngle, real_T throttlePercentage, real_T brakePercentage) = 0;

			virtual real_T GetEngineTorque(real_T wheelAngularVelocity) const = 0;

			virtual real_T GetBrakingTorque() const = 0;

			/*Gets the steering angle in radians. Angle is defined a counter-clockwise positive, with 0 being straight ahead*/
			virtual real_T GetSteeringAngleInRadians() const = 0;

			virtual real_T GetLongitudinalWheelBase() const = 0;
			virtual real_T GetLateralWheelBase() const = 0;

			virtual real_T GetCrossSectionalArea() const
			{
				return this->_staticParameters.CrossSectionalArea;
			}

			virtual real_T GetDragCoefficient() const
			{
				return this->_staticParameters.DragCoefficient;
			}

			/*Implementation of physicsbody*/
			virtual void kinematicsUpdated() override { this->_automobileState.Kinematics = this->getKinematics(); };
			virtual real_T getRestitution() const override { return 1; } //TODO: WTF is this?
			virtual real_T getFriction() const { return 0; } //We use a different friction model.

			virtual FreeBodyMotionAlgorithmType getFreeBodyMotionType() const override
			{
				/*If we are in the air, use projectile motion*/
				if (Utils::isDefinitelyGreaterThan(this->_automobileState.Kinematics.getPose().position.z(), 0.0f))
				{
					return MOTIONTYPE_PROJECTILE;
				}

				return MOTIONTYPE_AUTOMOBILE;
			}

			virtual std::vector<PneumaticWheel*> getWheels() const
			{
				return this->_wheels;
			}
	
		protected:
			AutomobileState _automobileState;
			AutomobileStaticParams _staticParameters;
			std::vector<PneumaticWheel*> _wheels;
			AutomobileEngine *_engine;
			AutomobileBrake *_brake;
	};

}}

#endif
#ifndef air_automobilesedan_hpp
#define air_automobilesedan_hpp

#include "automobile/Automobile.hpp"
#include "automobile/AutomobileStaticParams.hpp"
#include "common/common_utils/Utils.hpp"
#include "automobile/PneumaticWheel.hpp"
#include "automobile/AutomobileEngine.hpp"

namespace msr { namespace airlib {
	class AutomobileSedan : public Automobile {
		public:
			AutomobileSedan(const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment, std::vector<PneumaticWheel*> wheels, AutomobileEngine *engine, AutomobileBrake *brake, AutomobileStaticParams staticParameters);
			virtual void SetControlSignals(real_T steeringAngle, real_T throttlePercentage, real_T brakePercentage) override;
			virtual real_T GetEngineTorque(real_T wheelAngularVelocity) const override;
			virtual real_T GetBrakingTorque() const;
			virtual real_T GetSteeringAngleInRadians() const override;
			virtual real_T GetLongitudinalWheelBase() const override;
			virtual real_T GetLateralWheelBase() const override;
	};

}}


#endif
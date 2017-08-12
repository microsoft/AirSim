#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif
#include "automobile/MagicFormulaWheel.hpp"
#include "automobile/TirFileWheelParameters.hpp"

namespace msr {
	namespace airlib {

		MagicFormulaWheel::MagicFormulaWheel(TirFileWheelParameters tirWheelParameters, bool isPowered, bool hasBrake, bool canSteer)
			: PneumaticWheel(isPowered, hasBrake, canSteer)
		{
			this->_wheelParameters = tirWheelParameters;
		}

		Vector3r MagicFormulaWheel::GetForce(real_T wheelNormalForce, real_T directionalVelocity, real_T slipAngle, real_T coefficientOfFrictionMultiplier, TTimeDelta timeSinceLastUpdate)
		{
			this->UpdateAngularVelocity(timeSinceLastUpdate, wheelNormalForce, coefficientOfFrictionMultiplier);

			real_T longitudinalForce;
			real_T lateralForce;
			ComputeMagicFormulaForces(wheelNormalForce, slipAngle, directionalVelocity, coefficientOfFrictionMultiplier, longitudinalForce, lateralForce);
			return Vector3r(longitudinalForce, lateralForce, -1.0f * wheelNormalForce);
		}

		void MagicFormulaWheel::ComputeMagicFormulaForces(real_T wheelNormalForce,
			real_T slipAngle,
			real_T directionalVelocity,
			real_T coefficientOfFrictionMultiplier,
			real_T &longitudinalForce,
			real_T &lateralForce)
		{
			/*For the cars we are targeting, camber = 0 and the turn-slip is negligible.*/
			TirFileWheelParameters wp = this->_wheelParameters;

			real_T r_w = (wp.GetNoLoadOuterDiameter() / 2.0f) * this->_angularVelocity;
			real_T kappa = r_w - directionalVelocity;

			if (!Utils::isApproximatelyZero(kappa))
			{
				kappa /= std::max(std::abs(directionalVelocity), std::abs(r_w));
			}


			real_T gamma = 0; //TODO: get inclination angle with the road

			real_T fnomin = wp.FNOMIN * wp.LFZO;
			real_T dfz = (wheelNormalForce - fnomin) / fnomin;
			real_T dfzsq = dfz*dfz;

			/*Compute pure longitudinal slip*/
			real_T C_x = wp.PCX1 * wp.LCX;
			real_T mu_x = (wp.PDX1 + (wp.PDX2*dfz))*(1.0f - (wp.PDX3*gamma*gamma))*wp.LMUX;
			real_T D_x = mu_x * wheelNormalForce; 
			real_T S_H_x = (wp.PHX1 + (wp.PHX2*dfz))*wp.LHX;
			real_T S_V_x = wheelNormalForce * (wp.PVX1 + (wp.PVX2*dfz))*wp.LVX*wp.LMUX;

			//real_T gamma_x = gamma * wp.LGAX;
			real_T kappa_x = kappa + S_H_x;
			real_T E_x = (wp.PEX1 + (wp.PEX2*dfz) + (wp.PEX3*dfzsq))*(1.0f - (wp.PEX4*Utils::sgn(kappa_x)))*wp.LEX;
			real_T K_x = wheelNormalForce * (wp.PKX1 + (wp.PKX2*dfz))*exp(wp.PKX3*dfz)*wp.LKX;
			real_T B_x = K_x / (C_x*D_x);
			
			real_T F_x_0 = (D_x * sin((C_x * atan((B_x*kappa_x) - (E_x*((B_x*kappa_x) - atan(B_x*kappa_x))))))) + S_V_x;


			/*Compute the pure lateral slip*/
			real_T C_y = wp.PCY1 * wp.LCY;
			real_T gamma_y = gamma * wp.LGAY;
			real_T mu_y = (wp.PDY1 + (wp.PDY2*dfz))*(1.0f - (wp.PDY3*gamma_y*gamma_y)) * wp.LMUY;
			real_T D_y = mu_y * wheelNormalForce; 
			real_T S_H_y = ((wp.PHY1 + (wp.PHY2*dfz))*wp.LHY) + (wp.PHY3*gamma_y);
			real_T S_V_y = wheelNormalForce * (((wp.PVY1 + (wp.PVY2*dfz))*wp.LVY) + ((wp.PVY3 + (wp.PVY4*dfz))*gamma_y)) * wp.LMUY;

			real_T alpha_y = slipAngle + S_H_y;
			real_T E_y = (wp.PEY1 + (wp.PEY2*dfz))*(1.0f - ((wp.PEY3 + (wp.PEY4*gamma_y))*Utils::sgn(alpha_y)))*wp.LEY;

			real_T K_y_0 = wp.PKY1*fnomin*sin(2.0f*atan(wheelNormalForce / (wp.PKY2*fnomin*wp.LFZO)))*wp.LFZO*wp.LKY;
			real_T K_y = K_y_0 * (1.0f - (wp.PKY3*std::abs(gamma_y)));
			real_T B_y = K_y / (C_y*D_y);
			
			real_T F_y_0 = (D_y * sin(C_y * atan((B_y*alpha_y) - (E_y * ((B_y*alpha_y) - atan(B_y*alpha_y)))))) + S_V_y;

			/*Use cosine weighting function to determine combined longitudinal force*/
			real_T B_x_a = wp.RBX1 * cos(atan(wp.RBX2*kappa)) * wp.LXAL;
			real_T C_x_a = wp.RCX1;
			real_T S_H_x_a = wp.RHX1;
			real_T E_x_a = wp.REX1 + (wp.REX2*dfz);
			real_T alpha_s = slipAngle + S_H_x_a;
			//real_T D_x_a = F_x_0 / cos(C_x_a * atan((B_x_a*S_H_x_a) - (E_x_a*((B_x_a*S_H_x_a) - atan(B_x_a*S_H_x_a)))));

			real_T G_x_a = cos(C_x_a*atan((B_x_a*alpha_s) - (E_x_a* ((B_x_a*alpha_s) - atan(B_x_a*alpha_s)))));
			G_x_a /= cos(C_x_a*atan((B_x_a*S_H_x_a) - (E_x_a * ((B_x_a*S_H_x_a) - atan(B_x_a*S_H_x_a)))));
			longitudinalForce = F_x_0 * G_x_a;

			/*Use cosine weighting function to determine combined lateral force*/
			real_T C_y_k = wp.RCY1;
			real_T B_y_k = wp.RBY1 * cos(atan(wp.RBY2*(slipAngle - wp.RBY3))) * wp.LYKA;
			real_T S_H_y_k = wp.RHY1 + (wp.RHY2*dfz);
			//real_T D_V_y_k = (mu_y*wheelNormalForce)*(wp.RVY1 + (wp.RVY2*dfz) + (wp.RVY3*gamma))*cos(atan(wp.RVY4*slipAngle));
			//real_T S_V_y_k = D_V_y_k * sin(wp.RVY5*atan(wp.RVY6*kappa)) * wp.LVYKA;
			real_T E_y_k = wp.REY1 + (wp.REY2*dfz);
			//real_T D_y_k = F_y_0 / cos(C_y_k * atan((B_y_k*S_H_y_k) - (E_y_k * ((B_y_k*S_H_y_k) - atan(B_y_k * S_H_y_k)))));
			real_T kappa_s = kappa + S_H_y_k;

			real_T G_y_k = cos(C_y_k * atan((B_y_k*kappa_s) - (E_y_k * ((B_y_k*kappa_s) - atan(B_y_k*kappa_s)))));
			G_y_k /= cos(C_y_k * atan((B_y_k*S_H_y_k) - (E_y_k * ((B_y_k*S_H_y_k) - atan(B_y_k*S_H_y_k)))));
			lateralForce = F_y_0 * G_y_k;
		}

	}
}
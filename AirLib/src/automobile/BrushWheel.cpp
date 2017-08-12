#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif
#include "automobile/BrushWheel.hpp"

namespace msr {
	namespace airlib {

		BrushWheel::BrushWheel(BrushWheelParameters wheelParameters, bool isPowered, bool hasBrake, bool canSteer)
			: PneumaticWheel(isPowered, hasBrake, canSteer)
		{
			this->_wheelParameters = wheelParameters;
		}

		Vector3r BrushWheel::GetForce(real_T wheelNormalForce, real_T directionalVelocity, real_T slipAngle, real_T coefficientOfFrictionMultiplier, TTimeDelta timeSinceLastUpdate)
		{
			/*Begin by updating the wheel's velocity and accleration*/
			this->UpdateAngularVelocity(timeSinceLastUpdate, wheelNormalForce, coefficientOfFrictionMultiplier);

			/*Compute contact patch area*/
			real_T contactPatchWidth = this->ComputeContactPatchWidth(wheelNormalForce);
			real_T contactPatchLength = this->ComputeContactPatchLength(directionalVelocity, wheelNormalForce);

			/*compute the normalized forces*/
			real_T lateralForce;
			real_T longitudinalForce;

			this->ComputeBrushForces(wheelNormalForce,
				slipAngle,
				directionalVelocity,
				coefficientOfFrictionMultiplier,
				contactPatchWidth,
				contactPatchLength,
				longitudinalForce,
				lateralForce);

			/*Make return vector*/
			/*Assume that the wheel is unbreakable and will always be able to handle the normal force*/
			return Vector3r(longitudinalForce, lateralForce, -1.0f * wheelNormalForce);
		}

		std::vector<std::vector<real_T>> BrushWheel::TestForces()
		{
			std::vector<real_T> longTest;
			std::vector<real_T> kappas;
			std::vector<real_T> latTest;
			std::vector<real_T> alphas;

			real_T contactPatchWidth = .215f;
			real_T contactPatchHeight = .118f;
			real_T normalForce = 2200.0f;

			this->_angularVelocity = 1.0f;
			this->_wheelParameters.RIMDIAMETER = 1.0f;

			real_T r_w = (this->_wheelParameters.GetNoLoadOuterDiameter() / 2.0f) * this->_angularVelocity;
			real_T directionalVelocity = r_w * 0.0003f;
			while (directionalVelocity <= (r_w / 0.0003f))
			{
				real_T longForce;
				real_T latForce;
				ComputeBrushForces(normalForce, 0.0f, directionalVelocity, 1.0f, contactPatchWidth, contactPatchHeight, longForce, latForce);
				longTest.push_back(longForce / normalForce);

				real_T sigma_x = r_w - directionalVelocity;
				sigma_x /= std::max(std::abs(directionalVelocity), std::abs(r_w));

				kappas.push_back(sigma_x);
				directionalVelocity += 0.001f;
			}

			real_T alpha = -1.0f;
			while (alpha <= 1.0f)
			{
				real_T longForce;
				real_T latForce;
				ComputeBrushForces(2200.0f, alpha, r_w, 1.0f, contactPatchWidth, contactPatchHeight, longForce, latForce);
				latTest.push_back(latForce / 2200.0f);
				alphas.push_back(alpha);
				alpha += 0.01f;
			}

			std::vector<std::vector<real_T>> data;
			data.push_back(longTest);
			data.push_back(kappas);
			data.push_back(latTest);
			data.push_back(alphas);

			return data;
		}

		real_T BrushWheel::ComputeContactPatchWidth(real_T wheelNormalForce)
		{
			/*From https://bndtechsource.wixsite.com/home/tire-data-calculator */

			real_T crnrld = wheelNormalForce / this->_attachedAutomobile->getEnvironment().getState().gravity.norm();

			BrushWheelParameters wp = this->_wheelParameters;

			/*Compute design rim width*/
			/*This is the manufacturer's specification for the ideal rim width given the tire's aspect ratio and tire width*/
			/*per ERTRO dim a*/
			real_T drim;

			if (wp.ASPECTRATIO >= 75.0f)
				drim = wp.TIREWIDTH * 0.7f / 25.4f;
			else if (wp.ASPECTRATIO >= 60.0f)
				drim = wp.TIREWIDTH * 0.75f / 25.4f;
			else if (wp.ASPECTRATIO >= 50.0f)
				drim = wp.TIREWIDTH * 0.8f / 25.4f;
			else if (wp.ASPECTRATIO >= 45.0f)
				drim = wp.TIREWIDTH * 0.85f / 25.4f;
			else if (wp.ASPECTRATIO >= 30.0f)
				drim = wp.TIREWIDTH * 0.9f / 25.4f;
			else
				drim = wp.TIREWIDTH * 0.92f / 25.4f;

			/*Compare this with the actual rim width used*/
			real_T drimck = drim - wp.RIMWIDTH;

			/*Section width changes 5mm for every half-inch change in rim width*/
			real_T secwth = wp.TIREWIDTH + (5.0f * drimck); //TODO: Units?

			/*Theoretical rim width*/
			real_T thrim;
			if (wp.ASPECTRATIO > 50)
			{
				thrim = wp.TIREWIDTH * 0.7f;
			}
			else
			{
				thrim = wp.TIREWIDTH * 0.85f;
			}

			/*Section design width*/
			real_T secdesw = ((secwth + ((wp.RIMWIDTH * 25.4f) - thrim) * 0.4f));

			/*Contact patch static width*/
			real_T statcpw = (1.075f - (0.005f*wp.ASPECTRATIO)) * (std::pow(secdesw, 1.001f));

			/*Contact patch width given load and air pressure*/
			real_T contactPatchWidth = statcpw*(((-0.019355f*wp.AIRPRESSURE) + 0.04f) + 1.0f) * (((.000000079f*(crnrld*crnrld)) + (.000074023f*crnrld)) + 1.0f);
			return contactPatchWidth / 1000.0f;
		}

		real_T BrushWheel::ComputeContactPatchLength(real_T directionalVelocity, real_T wheelNormalForce)
		{
			/*From https://bndtechsource.wixsite.com/home/tire-data-calculator */

			real_T crnrld = wheelNormalForce / this->_attachedAutomobile->getEnvironment().getState().gravity.norm();

			BrushWheelParameters wp = this->_wheelParameters;

			/*Compute design rim width*/
			/*This is the manufacturer's specification for the ideal rim width given the tire's aspect ratio and tire width*/
			/*per ERTRO dim a*/
			real_T drim;

			if (wp.ASPECTRATIO >= 75.0f)
				drim = wp.TIREWIDTH * 0.7f / 25.4f;
			else if (wp.ASPECTRATIO >= 60.0f)
				drim = wp.TIREWIDTH * 0.75f / 25.4f;
			else if (wp.ASPECTRATIO >= 50.0f)
				drim = wp.TIREWIDTH * 0.8f / 25.4f;
			else if (wp.ASPECTRATIO >= 45.0f)
				drim = wp.TIREWIDTH * 0.85f / 25.4f;
			else if (wp.ASPECTRATIO >= 30.0f)
				drim = wp.TIREWIDTH * 0.9f / 25.4f;
			else
				drim = wp.TIREWIDTH * 0.92f / 25.4f;

			/*Compare this with the actual rim width used*/
			real_T drimck = drim - wp.RIMWIDTH;

			/*Sidewall height*/
			real_T h1 = wp.TIREWIDTH * wp.ASPECTRATIO / static_cast<real_T>(100.0);

			/*Sidewall width changes 2.5mm for every 0.5" change in rim width*/
			real_T secht = h1 + (2.5f * drimck); //TODO: units?

												 /*Rim diameter*/
			real_T dr = wp.RIMDIAMETER * 25.4f;


			/*Static radius at max load*/
			real_T rs1 = (0.5f * dr) + (wp.SIDEWALLDEFLECTION * secht);

			/*Outer diameter with no load*/
			real_T d = (2 * secht) + dr;

			/*spec. tire stiffness rate at max load and spec air pressure*/
			real_T stifrate = wp.LOADINDEX / ((d / 2.0f) - rs1);

			/*vehicle speed circumference factor from excel curve fitting equation*/
			real_T vscf = (0.000000003318f*pow(directionalVelocity, 3.0f)) - (0.000000003629f*pow(directionalVelocity, 2.0f)) + (0.000021348f*directionalVelocity) + 3.1416f;

			/*vehicle tire stiffness rate at veh speed and veh air pressure*/
			real_T vstifrate = (((((vscf*d) / 3.1416f) / 2.0f) / (d / 2.0f)) * (wp.AIRPRESSURE / 2.5f)) * stifrate;

			/*spec. tire stiffness rate at max load and spec air pressure*/
			real_T vdfl = crnrld / vstifrate;

			/*vehicle speed dynamic rolling radius at veh speed, veh load and veh air pressure*/
			real_T vsrd = (d / 2.0f) - vdfl;

			/*contact patch length*/
			real_T statcpl = sqrt(((d / 2.0f)*(d / 2.0f)) - (vsrd*vsrd)) * 2.0f;

			/*vehicle contact patch length at veh load and veh air pressure*/
			real_T contactPatchLength = statcpl * (((0.009874f*(wp.AIRPRESSURE*wp.AIRPRESSURE)) - (0.117384f*wp.AIRPRESSURE)) + 1.32f) * (((0.000000035f*(crnrld*crnrld)) + (0.000172792f*crnrld)) + 0.49f);
			return contactPatchLength / 1000.0f;
		}

		void BrushWheel::ComputeBrushForces(real_T wheelNormalForce,
			real_T slipAngle,
			real_T directionalVelocity,
			real_T coefficientOfFrictionMultiplier,
			real_T contactPatchWidth,
			real_T contactPatchLength,
			real_T &longitudinalForce,
			real_T &lateralForce)
		{

			/*From 13.8 in Rajesh Rajamani Vehicle Dynamics and Control*/
			/*For readability, the variable names will follow the naming convention in that segment*/
			real_T r_w = (this->_wheelParameters.GetNoLoadOuterDiameter() / 2.0f) * this->_angularVelocity;

			real_T sigma_x = r_w - directionalVelocity;

			real_T sigma_y = 0;

			/*Avoid divide by zero errors*/
			if (!Utils::isApproximatelyZero(r_w))
			{
				sigma_x /= std::max(std::abs(directionalVelocity), std::abs(r_w));
				sigma_y = (directionalVelocity / r_w) * tan(slipAngle);
			}
			else
			{
				sigma_x = static_cast<real_T>(0.0f);
				sigma_y = tan(slipAngle);
			}

			real_T a = contactPatchLength / 2.0f;
			real_T b = contactPatchWidth / 2.0f;
			real_T k = this->_wheelParameters.LATERALSTIFFNESSPERMETER / (contactPatchWidth);
			real_T mu = this->_wheelParameters.DRYSLIDINGFRICTIONCOEFFICIENT * coefficientOfFrictionMultiplier;

			real_T theta = 4.0f*a*a*b*k / (3.0f*mu*wheelNormalForce);

			real_T sigma_m = 1.0f / theta;

			real_T sigma = sqrt((sigma_x*sigma_x) + (sigma_y*sigma_y));

			if (Utils::isApproximatelyZero(sigma))
			{
				longitudinalForce = static_cast<real_T>(0.0f);
				lateralForce = static_cast<real_T>(0.0f);
				return;
			}

			/*TODO: Where does the factor of 10 come from?*/
			real_T f = this->_wheelParameters.DRYSLIDINGFRICTIONCOEFFICIENT * wheelNormalForce * coefficientOfFrictionMultiplier * 10.0f;

			if (sigma <= sigma_m)
			{
				real_T tts = 3.0f * theta * sigma;
				f *= tts - ((1.0f / 3.0f)*tts*tts) + ((1.0f / 27.0f)*tts*tts*tts);
			}

			longitudinalForce = (sigma_x / sigma) * f;
			lateralForce = (sigma_y / sigma) * f;
		}

	}
}
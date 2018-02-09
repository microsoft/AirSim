#ifndef msr_airsim_AdaptiveController_hpp
#define msr_airsim_AdaptiveController_hpp

#include "AdaptiveControllerParams.h"

//// ---- Name Spaces --------------------
using namespace simple_flight;
// -------------------------------------


		const IBoardClock* clock_;
		const int array_length = 7;
		float zero[array_length] = { 0 };
		float* adaptive_y = zero;
		float* adaptive_output = zero;

		//********************** SlidingModeModel Variables ******************************************/

		// state values
		float x_in, xdot_in, y_in, ydot_in, z_in, zdot_in, phi_in, P_in, theta_in, Q_in, psi_in, R_in;

		double x_des;
		double y_des;

		// State Vector
		double x[12][1];

		// References and trajectory values
		double refs_temp[4][1]; //temp vector for storing x,y,z,yaw refs
		double size_square = 4; // one side of the square is size_square/2
		double r[4][1]; // z, phi, theta, psi references into controller

		// update for angle states in SlidingModeModel
		double PQR[3][1], Angles[3][1], Angular_rates[3][1];
		double rollrate_ref, pitchrate_ref, yawrate_ref; //pc, qc, rc
		double delta_roll, delta_pitch, delta_yaw; // uncertainty parameters
		double S3_P, S3_Q, S3_R; //error in body frame angular rates


		// Iconfig Adaptive Sliding Variables
		double S2_phi, S2_theta, S2_psi; // error  in euler angles
		double delta_z, zdotdot; // uncertainty in z and calculated desired acceleration
		double delta_phi, delta_theta, delta_psi; // uncertainty in euler angles
		double R_matrix[3][3], R_inverse[3][3];

		// Iconfig Sliding Variables
		double refs[2][1], ref_angles[2][1]; // reference angles output from outer loop control

		Axis4r U_vec = 0;
		
		double U1, U2, U3, U4;

		double t = 0.0; //t variable

		/******* Rotate Body velocities to global frame *********/
		void Rotate_uvw_xyz(double u, double v, double w, double phi, double theta, double psi, double result[3][1])
		{
			result[0][0] = cos(theta)*cos(psi)*u + (-cos(phi)*sin(theta) + sin(phi)*sin(theta)*cos(phi))*v + (sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi))*w;
			result[1][0] = -cos(psi)*u + (cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi))*v + (-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi))*w;
			result[2][0] = -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;
		}
		void matrix_multiply_4_4_4_1(double mat1[4][4], double mat2[4][1], double result[4][1])
		{
			int i, j, k;
			double sum;
			for (i = 0; i<4; i++)
			{
				for (j = 0; j<1; j++)
				{
					sum = 0;
					for (k = 0; k<4; k++)
					{
						sum += mat1[i][k] * mat2[k][j];
					}
					result[i][j] = sum;
				}
			}
		}


		void Sliding_Iconfig(double zddot, double x_desired, double x_current, double xdot, double y_desired, double y_current, double ydot, double yaw_current, double result[2][1])
		{
			double Fz, xddot, yddot;
			int i;
			Fz = (zddot + 9.81)*0.9116;
			xddot = -2.0 * xdot - 2.0 * (x_current - x_desired);
			yddot = -2.0 * ydot - 3.0 * (y_current - y_desired);
			if (Fz == 0) {
				Fz = 9.81*0.9116;
			}
			result[0][0] = (xddot*sin(yaw_current) - yddot*cos(yaw_current))*0.9116 / Fz;
			result[1][0] = (xddot*cos(yaw_current) + yddot*sin(yaw_current))*0.9116 / Fz;
			// Limit the angle reference values
			for (i = 0; i < 2; i++) {
				if (result[i][0] > 0.10) {
					result[i][0] = 0.10;
				}
				else if (result[i][0] < -0.10) {
					result[i][0] = -0.10;
				}

			}
		}

		void square_trajectory(double t, double result[4][1], double height)
		{
			t = t - 50;
			if (t <= 120)
			{

				if (t > 0) {
					if (t < 10) {
						result[0][0] = 0;
					}
					if ((t >= 10) && (t <= 14)) {
						result[0][0] = 2 * (t - 10) / size_square;
					}
					if ((t > 14) && (t < 18)) {
						result[0][0] = 2;
					}
					if ((t >= 18) && (t <= 26)) {
						result[0][0] = 2 - 2 * (t - 18) / size_square;
					}
					if ((t > 26) && (t < 34)) {
						result[0][0] = -2;
					}
					if ((t >= 34) && (t <= 42)) {
						result[0][0] = -2 + 2 * (t - 34) / size_square;
					}
					if ((t > 42) && (t < 50)) {
						result[0][0] = 2;
					}
					if ((t >= 50) && (t <= 58)) {
						result[0][0] = 2 - 2 * (t - 50) / size_square;
					}
					if ((t > 58) && (t < 66)) {
						result[0][0] = -2;
					}
					if ((t >= 66) && (t <= 74)) {
						result[0][0] = -2 + 2 * (t - 66) / size_square;
					}
					if ((t > 74) && (t < 82)) {
						result[0][0] = 2;
					}
					if ((t >= 82) && (t <= 90)) {
						result[0][0] = 2 - 2 * (t - 82) / size_square;
					}
					if ((t > 90) && (t < 98)) {
						result[0][0] = -2;
					}
					if ((t >= 98) && (t <= 106)) {
						result[0][0] = -2 + 2 * (t - 98) / size_square;
					}
					if ((t > 106) && (t < 114)) {
						result[0][0] = 2;
					}
				}
				if (t < 10) {
					result[1][0] = 0;
				}
				if ((t >= 10) && (t <= 14)) {
					result[1][0] = 0;
				}
				if ((t > 14) && (t < 18)) {
					result[1][0] = 2 * (t - 14) / size_square;
				}
				if ((t >= 18) && (t <= 26)) {
					result[1][0] = 2;
				}
				if ((t > 26) && (t < 34)) {
					result[1][0] = 2 - 2 * (t - 26) / size_square;
				}
				if ((t >= 34) && (t <= 42)) {
					result[1][0] = -2;
				}
				if ((t > 42) && (t < 50)) {
					result[1][0] = -2 + 2 * (t - 42) / size_square;
				}
				if ((t >= 50) && (t <= 58)) {
					result[1][0] = 2;
				}
				if ((t > 58) && (t < 66)) {
					result[1][0] = 2 - 2 * (t - 58) / size_square;
				}
				if ((t >= 66) && (t <= 74)) {
					result[1][0] = -2;
				}
				if ((t > 74) && (t < 82)) {
					result[1][0] = -2 + 2 * (t - 74) / size_square;
				}
				if ((t >= 82) && (t <= 90)) {
					result[1][0] = 2;
				}
				if ((t > 90) && (t < 98)) {
					result[1][0] = 2 - 2 * (t - 90) / size_square;
				}
				if ((t >= 98) && (t <= 106)) {
					result[1][0] = -2;
				}
				if ((t > 106) && (t < 110)) {
					result[1][0] = -2 + 2 * (t - 106) / size_square;
				}
				result[0][0] = result[0][0] + 6.91; // add x offset for center of vicon space
				result[1][0] = result[1][0] + 4.01; // add y offset for center of vicon space
				result[2][0] = height; // z_ref
				result[3][0] = 0; // yaw ref

			}
			else
			{
				result[0][0] = 6.91;
				result[1][0] = 4.01;
				result[2][0] = height;
				result[3][0] = 0;
			}
			t = t + 50;

		}

		void circle_trajectory(double t, double circle_radius, double height, double result[4][1])
		{
			if (t < 20) {
				result[0][0] = 2.5;
				result[1][0] = 0;
				result[2][0] = height;
				result[3][0] = 0;
			}
			else if (t > 20) {
				result[0][0] = circle_radius * cos(.22*(t - 20)) ; 
				result[1][0] = circle_radius * sin(.22*(t - 20)) ; 
				result[2][0] = height;
				result[3][0] = 0;
			}
		}

		void step_response_3D(double t, double result[4][1])
		{
			if (t < 30) {
				result[0][0] = 0;
				result[1][0] = 0;
				result[2][0] = 1;
				result[3][0] = 0;
			}
			else if (t > 30) {
				result[0][0] = 10.5;
				result[1][0] = 0;
				result[2][0] = 2.5;
				result[3][0] = 0;
			}
		}

		void inverse_3x3(double A[3][3], double result[3][3])
		{
			double det_A; // dummy variable
			det_A = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) + A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2])*A[2][0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
			if (det_A == 0) {
				result[0][0] = 0;
				result[0][1] = 0;
				result[0][2] = 0;
				result[1][0] = 0;
				result[1][1] = 0;
				result[1][2] = 0;
				result[2][0] = 0;
				result[2][1] = 0;
				result[2][2] = 0;
			}
			else {
				result[0][0] = (1 / det_A)*(A[1][1] * A[2][2] - A[1][2] * A[2][1]);
				result[0][1] = (1 / det_A)*(A[0][2] * A[2][1] - A[0][1] * A[2][2]);
				result[0][2] = (1 / det_A)*(A[0][1] * A[1][2] - A[0][2] * A[1][1]);
				result[1][0] = (1 / det_A)*(A[1][2] * A[2][0] - A[1][0] * A[2][2]);
				result[1][1] = (1 / det_A)*(A[0][0] * A[2][2] - A[0][2] * A[2][0]);
				result[1][2] = (1 / det_A)*(A[0][2] * A[1][0] - A[0][0] * A[1][2]);
				result[2][0] = (1 / det_A)*(A[1][0] * A[2][1] - A[1][1] * A[2][0]);
				result[2][1] = (1 / det_A)*(A[0][1] * A[2][0] - A[0][0] * A[2][1]);
				result[2][2] = (1 / det_A)*(A[0][0] * A[1][1] - A[0][1] * A[1][0]);
			}
		}

		void PQR_generation(double states[12][1], double result[3][1])
		{
			result[0][0] = states[7][0] - sin(states[6][0])*states[10][0];
			result[1][0] = states[9][0] * cos(states[6][0]) + states[10][0] * (cos(states[8][0])*sin(states[6][0]));
			result[2][0] = states[11][0] * (cos(states[8][0])*cos(states[6][0])) - states[9][0] * sin(states[6][0]);
		}

		void Angular_velocities_from_PQR(double PQR[3][1], double Angles[3][1], double result[3][1])
		{
			result[0][0] = PQR[0][0] + PQR[1][0] * sin(Angles[0][0])*tan(Angles[1][0]) + PQR[2][0] * cos(Angles[0][0])*tan(Angles[1][0]);
			result[1][0] = PQR[1][0] * cos(Angles[0][0]) - PQR[2][0] * sin(Angles[0][0]);
			result[2][0] = PQR[1][0] * (sin(Angles[0][0]) / cos(Angles[1][0])) + PQR[2][0] * (cos(Angles[0][0]) / cos(Angles[1][0]));
		}
		

		// ---------------------------------------------------------------------------
		// Local Support Functions
		// ---------------------------------------------------------------------------

		uint64_t getTimeU()
		{
			uint64_t last_time_ = clock_ == nullptr ? 0 : clock_->millis();
			return last_time_;
		}


		void remapU(double control_u1, double control_u2, double control_u3, double control_u4)
		{
			// Map to px4 U outputs
			U1 = control_u2; // roll
			U2 = -control_u3; // pitch
			U3 = control_u4; // yaw
			U4 = control_u1; // throttle
			U_vec[0] = U1;
			U_vec[1] = U2;
			U_vec[2] = U3;
			U_vec[3] = U4;
		}  


		// ---------------------------------------------------------------------------
		// SlidingModeModel method
		// ---------------------------------------------------------------------------
		void model(float* y, uint64_t last_time_, float* y_out)
		{
			/**********desired trajectory************************/


			circle_trajectory(t, radius, alt, refs_temp);
			//square_trajectory(t, refs_temp, alt);
			//step_response_3D(t, refs_temp);

			x_des = refs_temp[0][0];  
			y_des = refs_temp[1][0];  
			r[0][0] = 2.5;   //z/
			if (t < 10)
			{
				r[0][0] = 0.25 * t;
			}
			r[3][0] = 0;   //yaw (rad)

			float lambda_theta = 1;
			float lambda_theta_rate = 0.06;
			float lambda_phi = 0.5;
			float lambda_phi_rate = 0.05;
			float lambda_psi = 0.4;
			float lambda_psi_rate = 0.05;
			float lambda_z = .5;//

			
			

			/********** Input the State Vector***********/
			x[0][0] = x_in;                        
			x[1][0] = xdot_in;
			x[2][0] = y_in;
			x[3][0] = ydot_in;
			x[4][0] = z_in;
			x[5][0] = zdot_in;
			x[6][0] = phi_in;
			Angles[0][0] = phi_in;
			PQR[0][0] = P_in;
			x[8][0] = theta_in;
			Angles[1][0] = theta_in;
			PQR[1][0] = Q_in;
			x[10][0] = psi_in;
			Angles[2][0] = psi_in;
			PQR[2][0] = R_in;


			Angular_velocities_from_PQR(PQR, Angles, Angular_rates);
			x[7][0] = Angular_rates[0][0];
			x[9][0] = Angular_rates[1][0];
			x[11][0] = Angular_rates[2][0];

			// -4, -10 are adjustable gains for the z controller
			zdotdot = -6 * x[5][0] - 15* (x[4][0] - r[0][0]);

			/**********Iconfig Sliding Mode***********************/

			Sliding_Iconfig(zdotdot, x_des, x[0][0], x[1][0], y_des, x[2][0], x[3][0], x[10][0], refs);
			ref_angles[0][0] = refs[0][0];
			ref_angles[1][0] = refs[1][0];


			/* Reference angles to be sent to inner loop */
			r[1][0] = ref_angles[0][0]; //phi ref
			r[2][0] = ref_angles[1][0]; //theta ref

			/************ Iconfig Control Law *********************/
			// First get integrated uncertainty parameters
			delta_z = y[6];
			delta_phi = y[0];
			delta_theta = y[1];
			delta_psi = y[2];
			delta_roll = y[3];
			delta_pitch = y[4];
			delta_yaw = y[5];


			U1 = (zdotdot + grav)*m + delta_z;
			
			y_out[6] = lambda_z*zdotdot; // generate sliding surface in z, .015 is adjustable slope for sliding surface

			// error in euler angles
			S2_phi = x[6][0] - r[1][0];
			S2_theta = x[8][0] - r[2][0];
			S2_psi = x[10][0] - r[3][0];

			//cout << x[0][0] << " " << x[1][0] << " " << x[2][0] << " " << x[3][0] << " " << x[4][0] << " " << x[5][0] << " " << x[6][0] << " " << x[7][0] << " " << x[8][0] << " " << x[9][0] << " " << x[10][0] << " " << x[11][0] << " " << "\n";

			// generate delta_dot which goes to integrator variable, sliding surface for 3 euler angles, can adjust sliding surface slope as desired
			y_out[0] = lambda_phi*S2_phi;
			y_out[1] = lambda_theta*S2_theta;
			y_out[2] = lambda_psi*S2_psi;

			R_matrix[0][0] = 1;
			R_matrix[1][0] = sin(x[6][0] * tan(x[8][0]));
			R_matrix[2][0] = cos(x[6][0])*tan(x[8][0]);
			R_matrix[1][0] = 0;
			R_matrix[1][1] = cos(x[6][0]);
			R_matrix[1][2] = -1 * sin(x[6][0]);
			R_matrix[2][0] = 0;
			R_matrix[2][1] = sin(x[6][0]) / cos(x[8][0]);
			R_matrix[2][2] = cos(x[6][0]) / cos(x[8][0]);

			inverse_3x3(R_matrix, R_inverse);
			rollrate_ref = R_inverse[0][0] * S2_phi*k_phi + R_inverse[0][1] * S2_theta*k_theta + R_inverse[0][2] * S2_psi*k_psi - delta_phi;
			pitchrate_ref = R_inverse[1][0] * S2_phi*k_phi + R_inverse[1][1] * S2_theta*k_theta + R_inverse[1][2] * S2_psi*k_psi - delta_theta;
			yawrate_ref = R_inverse[2][0] * S2_phi*k_phi + R_inverse[2][1] * S2_theta*k_theta + R_inverse[2][2] * S2_psi*k_psi - delta_psi;

			PQR_generation(x, PQR);
			S3_P = PQR[0][0] - rollrate_ref;
			S3_Q = PQR[1][0] - pitchrate_ref;
			S3_R = PQR[2][0] - yawrate_ref;

			// Sliding surface for the body frame angular rates to be integrated
			y_out[3] = lambda_phi_rate*S3_P;
			y_out[4] = lambda_theta_rate*S3_Q;
			y_out[5] = lambda_psi_rate*S3_R;

			// Calculate controls for roll, pitch, and yaw
			U2 = k_roll*S3_P*Ix + (Iz - Iy)*PQR[1][0] * PQR[2][0] - delta_roll;
			U3 = k_pitch*S3_Q*Iy + (Ix - Iz)*PQR[0][0] * PQR[2][0] - delta_pitch;
			U4 = k_yaw*S3_R*Iz + (Iy - Ix)*PQR[0][0] * PQR[1][0] - delta_yaw;


			// Rescale such that the outputs normalize from -1,1
			
			U1 =  sqrt(abs(U1)) / 6.20; // I used sqrt to try and allow for smoother signal

			U2 =  U2 / 80;

			U3 =  U3 / 80;

			U4 =  U4 / 80;


			// Saturations: U1->.35,1 : U2,U3,U4 -> -.2,.2
			if (U1 > 1)
			{
				U1 = 1;
			}
			else if (U1 < 0.35)
			{
				U1 = 0.35;
			}
			if (U2 > U2_sat)
			{
				U2 = U2_sat;
			}
			else if (U2 < -U2_sat)
			{
				U2 = -U2_sat;
			}
			if (U3 > U3_sat)
			{
				U3 = U3_sat;
			}
			else if (U3 < -U3_sat)
			{
				U3 = -U3_sat;
			}
			if (U4 > U4_sat)
			{
				U4 = U4_sat;
			}
			else if (U4 < -U4_sat)
			{
				U4 = -U4_sat;
			}
			
			remapU(U1, U2, U3, U4); //remap to axis4r


		} // SlidingModeModel */

		  // =================================================================
		  /*************** Test Class Implementations *****************/
		  // =================================================================

		// ----------------------------------------------------------------------------------
		// Update State Vals
		// ----------------------------------------------------------------------------------
		void AdaptiveController::update_state_vals(float x, float vx, float y, float vy, float z, float vz, float roll, float roll_rate, float pitch, float pitch_rate, float yaw, float yaw_rate)
		{
			x_in = x;
			xdot_in = vx;
			// negative conventions in the y and z directions are a result of vicon space coordinate needs
			y_in = -y;
			ydot_in = -vy;
			z_in = -z;
			zdot_in = -vz;
			phi_in = roll;
			P_in = roll_rate;
			theta_in = -pitch;
			Q_in = -pitch_rate;
			psi_in = yaw;
			R_in = yaw_rate; 


			// bias modification for level imu implementing deadband

			if (abs(phi_in) <= 0.0001)
				phi_in = 0;

			if (abs(theta_in) <= 0.00001)
				theta_in = 0;

			if (abs(psi_in) <= 0.0001)
				psi_in = 0;
		}

		void rungeKutta(float* y, float* yp, uint64_t t, float dt, int size)
		{	
			float zero_vec[array_length] = { 0 };
			float k1[array_length] = { 0 };
			float k2[array_length] = { 0 };
			float k3[array_length] = { 0 };
			float k4[array_length] = { 0 };
			float* y_temp;
			float* y_out = zero_vec;
			y_temp = y;
			model(y_temp, t, y_out);
			for (int n = 0; n < size; n++)
			{
				k1[n] = dt*y_out[n];
				y_temp[n] = y[n] + k1[n] / 2;
			}
			model(y_temp, t + dt / 2, y_out);
			for (int n = 0; n < size; n++)
			{
				k2[n] = dt*y_out[n];
				y_temp[n] = y[n] + k2[n] / 2;
			}
			model(y_temp, t + dt / 2, y_out);
			for (int n = 0; n < size; n++)
			{
				k3[n] = dt*y_out[n];
				y_temp[n] = y[n] + k3[n];
			}
			model(y_temp, t + dt, y_out);
			for (int n = 0; n < size; n++)
			{
				k4[n] = dt*y_out[n];
				yp[n] = y[n] + k1[n] / 6 + k2[n] / 3 + k3[n] / 4 + k4[n] / 6;
			}
		}

		// ----------------------------------------------------------------------------------
		// Run
		// ----------------------------------------------------------------------------------

		void AdaptiveController::run()
		{
			rungeKutta(adaptive_y, adaptive_output, getTimeU(), 0.003, array_length);
		}

		// -------------------------------------------------------------------------------------
		// Accessor Functions
		// -------------------------------------------------------------------------------------

		double AdaptiveController::get_U1() //pitch
		{
			return U1;
		}

		double AdaptiveController::get_U2() //roll
		{
			return U2;
		}

		double AdaptiveController::get_U3() //thrust
		{
			return U3;
		}

		double AdaptiveController::get_U4() //yaw
		{
			return U4;
		}
		
#endif
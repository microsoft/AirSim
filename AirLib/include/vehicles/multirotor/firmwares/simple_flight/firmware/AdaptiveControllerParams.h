/*
// This subroutine has been coded by:
// Mr. C. Domann
// Advanced Control Systems Lab
// The University of Oklahoma
// Advisor: Dr. A. L'Afflitto
*/

// --------------------------------------------------------------------------
// DEFINES (drone based defines for physical characteristics)
// --------------------------------------------------------------------------

//inertia parameters
#define Ix  0.02 
#define Iy  0.01
#define Iz  0.03
 
#define l  0.11 //arm length, can make more accurate by being specific about lx, ly                  
#define m  0.916 // mass in kg                                                                 
#define grav  9.81 // gravity
#define Jr  0.00006 // inertia of rotor, currently an estimate; make more accurate by getting a measured value

// Static Gain Variables
#define k_phi  -16.75 // roll angle
#define k_theta  -26.75 //  pitch angle
#define k_psi  -13 //  yaw angle
#define k_roll  -450 // roll rate
#define k_pitch  -450 // pitch rate
#define k_yaw  -400 // yaw rate

//input saturation
#define U1_sat  1
#define U2_sat  .95
#define U3_sat  .95
#define U4_sat  .95

//trajectory parameters
#define pi  3.14159265359
#define period  45
#define radius  2.5 // input radius of the circle
#define alt  5 // height used for circle/square

// other constants
#define SIZE_BUF_FLOAT 30
#define NEQN 7

// ---------------------------------------------------------------------------------------------------
// Prototype Functions
// ---------------------------------------------------------------------------------------------------

// support functions
void Rotate_uvw_xyz(double u, double v, double w, double phi, double theta, double psi, double result[3][1]);
void Sliding_Iconfig(double zddot, double x_desired, double x_current, double xdot, double y_desired, double y_current, double ydot, double yaw_current, double result[2][1]);
void step_response_3D(double time, double result[4][1]);
void inverse_3x3(double A[3][3], double result[3][3]);
void PQR_generation(double states[12][1], double result[3][1]);
void Angular_velocities_from_PQR(double PQR[3][1], double Angles[3][1], double result[3][1]);
void matrix_multiply_4_4_4_1(double mat1[4][4], double mat2[4][1], double result[4][1]);
uint64_t getTimeU();



// ---------------------------------------------------------------------------------------------------
// Adaptive Controller Class
// ---------------------------------------------------------------------------------------------------
class AdaptiveController
{

public:

	void update_state_vals(float x, float vx, float y, float vy, float z, float vz, float roll, float roll_rate, float pitch, float pitch_rate, float yaw, float yaw_rate);
	void run();

	double get_U1();
	double get_U2();
	double get_U3();
	double get_U4();
	


private:


}; 

//#endif /* CONTROLLERS_TEST_CONTROLLER_H_ */

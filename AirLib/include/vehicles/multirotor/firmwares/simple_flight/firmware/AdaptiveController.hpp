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


#ifndef msr_airsim_AdaptiveController_hpp
#define msr_airsim_AdaptiveController_hpp


namespace simple_flight {


class AdaptiveController : public IController {
public:

    virtual void initialize(const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        //TODO: goal is never used!
        goal_ = goal;
        state_estimator_ = state_estimator;
    }

    virtual const Axis4r& getOutput() override
    {
        // Create vectors to store state variables
        Axis3r angle = 0;
        Axis3r position = 0;
        Axis3r angle_rate = 0; 
        Axis3r velocity = 0;

        // Assign state variables to placeholder variables
        angle = state_estimator_->getAngles();
        angle_rate = state_estimator_->getAngulerVelocity();
        position = state_estimator_->getPosition();
        velocity = state_estimator_->getLinearVelocity();

        // Send state variables to the adaptive controller
        update_state_vals(position[0], velocity[0], position[1], velocity[1], position[2], velocity[2], 
            angle[0], angle_rate[0], angle[1], angle_rate[1], angle[2], angle_rate[2]);

        run();

        // Replace PID control variables with Adaptive control variables
        output_controls_[0] = get_U1();
        output_controls_[1] = get_U2();
        output_controls_[2] = get_U3();
        output_controls_[3] = get_U4();

        return output_controls_;
    }

private:
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;
    Axis4r output_controls_;

private:
    //inertia parameters
    const float Ix = 0.02f;
    const float Iy = 0.01f;
    const float Iz = 0.03f;

    const float l = 0.11f; //arm length, can make more accurate by being specific about lx, ly                  
    const float m = 0.916f; // mass in kg                                                                 
    const float grav = 9.81f; // gravity
    const float Jr = 0.00006f; // inertia of rotor, currently an estimate; make more accurate by getting a measured value

    // Static Gain Variables
    const float k_phi = -16.75f; // roll angle
    const float k_theta =  -26.75f; //  pitch angle
    const float k_psi = -13.0f; //  yaw angle
    const float k_roll = -450.0f; // roll rate
    const float k_pitch = -450.0f; // pitch rate
    const float k_yaw = -400.0f; // yaw rate

    //input saturation
    const float U1_sat = 1.0f;
    const float U2_sat = .95f;
    const float U3_sat = .95f;
    const float U4_sat = .95f;

    //trajectory parameters
    const float pi = 3.14159265359f;
    const float period = 45.0f;
    const float radius = 2.5f; // input radius of the circle
    const float alt = 5.0f; // height used for circle/square

    // other constants
    const float SIZE_BUF_FLOAT = 30.0f;
    const float NEQN = 7.0f;

private:
    void update_state_vals(float x_val, float vx, float y_val, float vy, float z_val, float vz, float roll, float roll_rate, float pitch, float pitch_rate, float yaw, float yaw_rate)
    {
        x_in = x_val;
        xdot_in = vx;
        // negative conventions in the y and z directions are a result of vicon space coordinate needs
        y_in = -y_val;
        ydot_in = -vy;
        z_in = -z_val;
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

    void run()
    {
        rungeKutta(adaptive_y, adaptive_output, getTimeU(), 0.003f, array_length);
    }

    float get_U1() //pitch
    {
        return static_cast<float>(U1);
    }

    float get_U2() //roll
    {
        return static_cast<float>(U2);
    }

    float get_U3() //thrust
    {
        return static_cast<float>(U3);
    }

    float get_U4() //yaw
    {
        return static_cast<float>(U4);
    }

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

    void square_trajectory(double t_val, double result[4][1], double height)
    {
        t_val = t_val - 50;
        if (t_val <= 120)
        {

            if (t_val > 0) {
                if (t_val < 10) {
                    result[0][0] = 0;
                }
                if ((t_val >= 10) && (t_val <= 14)) {
                    result[0][0] = 2 * (t_val - 10) / size_square;
                }
                if ((t_val > 14) && (t_val < 18)) {
                    result[0][0] = 2;
                }
                if ((t_val >= 18) && (t_val <= 26)) {
                    result[0][0] = 2 - 2 * (t_val - 18) / size_square;
                }
                if ((t_val > 26) && (t_val < 34)) {
                    result[0][0] = -2;
                }
                if ((t_val >= 34) && (t_val <= 42)) {
                    result[0][0] = -2 + 2 * (t_val - 34) / size_square;
                }
                if ((t_val > 42) && (t_val < 50)) {
                    result[0][0] = 2;
                }
                if ((t_val >= 50) && (t_val <= 58)) {
                    result[0][0] = 2 - 2 * (t_val - 50) / size_square;
                }
                if ((t_val > 58) && (t_val < 66)) {
                    result[0][0] = -2;
                }
                if ((t_val >= 66) && (t_val <= 74)) {
                    result[0][0] = -2 + 2 * (t_val - 66) / size_square;
                }
                if ((t_val > 74) && (t_val < 82)) {
                    result[0][0] = 2;
                }
                if ((t_val >= 82) && (t_val <= 90)) {
                    result[0][0] = 2 - 2 * (t_val - 82) / size_square;
                }
                if ((t_val > 90) && (t_val < 98)) {
                    result[0][0] = -2;
                }
                if ((t_val >= 98) && (t_val <= 106)) {
                    result[0][0] = -2 + 2 * (t_val - 98) / size_square;
                }
                if ((t_val > 106) && (t_val < 114)) {
                    result[0][0] = 2;
                }
            }
            if (t_val < 10) {
                result[1][0] = 0;
            }
            if ((t_val >= 10) && (t_val <= 14)) {
                result[1][0] = 0;
            }
            if ((t_val > 14) && (t_val < 18)) {
                result[1][0] = 2 * (t_val - 14) / size_square;
            }
            if ((t_val >= 18) && (t_val <= 26)) {
                result[1][0] = 2;
            }
            if ((t_val > 26) && (t_val < 34)) {
                result[1][0] = 2 - 2 * (t_val - 26) / size_square;
            }
            if ((t_val >= 34) && (t_val <= 42)) {
                result[1][0] = -2;
            }
            if ((t_val > 42) && (t_val < 50)) {
                result[1][0] = -2 + 2 * (t_val - 42) / size_square;
            }
            if ((t_val >= 50) && (t_val <= 58)) {
                result[1][0] = 2;
            }
            if ((t_val > 58) && (t_val < 66)) {
                result[1][0] = 2 - 2 * (t_val - 58) / size_square;
            }
            if ((t_val >= 66) && (t_val <= 74)) {
                result[1][0] = -2;
            }
            if ((t_val > 74) && (t_val < 82)) {
                result[1][0] = -2 + 2 * (t_val - 74) / size_square;
            }
            if ((t_val >= 82) && (t_val <= 90)) {
                result[1][0] = 2;
            }
            if ((t_val > 90) && (t_val < 98)) {
                result[1][0] = 2 - 2 * (t_val - 90) / size_square;
            }
            if ((t_val >= 98) && (t_val <= 106)) {
                result[1][0] = -2;
            }
            if ((t_val > 106) && (t_val < 110)) {
                result[1][0] = -2 + 2 * (t_val - 106) / size_square;
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
        t_val = t_val + 50;

    }

    void circle_trajectory(double t_val, double circle_radius, double height, double result[4][1])
    {
        if (t_val < 20) {
            result[0][0] = 2.5;
            result[1][0] = 0;
            result[2][0] = height;
            result[3][0] = 0;
        }
        else if (t_val > 20) {
            result[0][0] = circle_radius * cos(.22*(t_val - 20)) ; 
            result[1][0] = circle_radius * sin(.22*(t_val - 20)) ; 
            result[2][0] = height;
            result[3][0] = 0;
        }
    }

    void step_response_3D(double t_val, double result[4][1])
    {
        if (t_val < 30) {
            result[0][0] = 0;
            result[1][0] = 0;
            result[2][0] = 1;
            result[3][0] = 0;
        }
        else if (t_val > 30) {
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

    void Angular_velocities_from_PQR(double PQR_val[3][1], double Angles_val[3][1], double result[3][1])
    {
        result[0][0] = PQR_val[0][0] + PQR_val[1][0] * sin(Angles_val[0][0])*tan(Angles_val[1][0]) + PQR_val[2][0] * cos(Angles_val[0][0])*tan(Angles_val[1][0]);
        result[1][0] = PQR_val[1][0] * cos(Angles_val[0][0]) - PQR_val[2][0] * sin(Angles_val[0][0]);
        result[2][0] = PQR_val[1][0] * (sin(Angles_val[0][0]) / cos(Angles_val[1][0])) + PQR_val[2][0] * (cos(Angles_val[0][0]) / cos(Angles_val[1][0]));
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
        U_vec[0] = static_cast<float>(U1);
        U_vec[1] = static_cast<float>(U2);
        U_vec[2] = static_cast<float>(U3);
        U_vec[3] = static_cast<float>(U4);
    }  

    void rungeKutta(float* y, float* yp, uint64_t t_val, float dt, int size)
    {	
        float zero_vec[array_length] = { 0 };
        float k1[array_length] = { 0 };
        float k2[array_length] = { 0 };
        float k3[array_length] = { 0 };
        float k4[array_length] = { 0 };
        float* y_temp;
        float* y_out = zero_vec;
        y_temp = y;
        model(y_temp, static_cast<float>(t_val), y_out);
        for (int n = 0; n < size; n++)
        {
            k1[n] = dt*y_out[n];
            y_temp[n] = y[n] + k1[n] / 2;
        }
        model(y_temp, t_val + dt / 2, y_out);
        for (int n = 0; n < size; n++)
        {
            k2[n] = dt*y_out[n];
            y_temp[n] = y[n] + k2[n] / 2;
        }
        model(y_temp, t_val + dt / 2, y_out);
        for (int n = 0; n < size; n++)
        {
            k3[n] = dt*y_out[n];
            y_temp[n] = y[n] + k3[n];
        }
        model(y_temp, t_val + dt / 2, y_out);
        for (int n = 0; n < size; n++)
        {
            k4[n] = dt*y_out[n];
            yp[n] = y[n] + k1[n] / 6 + k2[n] / 3 + k3[n] / 4 + k4[n] / 6;
        }
    }

    // ---------------------------------------------------------------------------
    // SlidingModeModel method
    // ---------------------------------------------------------------------------
    void model(float* y, float last_time_, float* y_out)
    {
        unused(last_time_);

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
        float lambda_theta_rate = 0.06f;
        float lambda_phi = 0.5f;
        float lambda_phi_rate = 0.05f;
        float lambda_psi = 0.4f;
        float lambda_psi_rate = 0.05f;
        float lambda_z = .5f;


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

        y_out[6] = static_cast<float>(lambda_z*zdotdot); // generate sliding surface in z, .015 is adjustable slope for sliding surface

                                     // error in euler angles
        S2_phi = x[6][0] - r[1][0];
        S2_theta = x[8][0] - r[2][0];
        S2_psi = x[10][0] - r[3][0];

        //cout << x[0][0] << " " << x[1][0] << " " << x[2][0] << " " << x[3][0] << " " << x[4][0] << " " << x[5][0] << " " << x[6][0] << " " << x[7][0] << " " << x[8][0] << " " << x[9][0] << " " << x[10][0] << " " << x[11][0] << " " << "\n";

        // generate delta_dot which goes to integrator variable, sliding surface for 3 euler angles, can adjust sliding surface slope as desired
        y_out[0] = static_cast<float>(lambda_phi*S2_phi);
        y_out[1] = static_cast<float>(lambda_theta*S2_theta);
        y_out[2] = static_cast<float>(lambda_psi*S2_psi);

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
        y_out[3] = static_cast<float>(lambda_phi_rate*S3_P);
        y_out[4] = static_cast<float>(lambda_theta_rate*S3_Q);
        y_out[5] = static_cast<float>(lambda_psi_rate*S3_R);

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

private:
    const IBoardClock* clock_;
    static constexpr int array_length = 7;
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

}; 
    

}
#endif
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

namespace simple_flight
{

class AdaptiveController : public IController
{
public:
    virtual void initialize(const IGoal* goal, const IStateEstimator* state_estimator) override
    {
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
        angle_rate = state_estimator_->getAngularVelocity();
        position = state_estimator_->getPosition();
        velocity = state_estimator_->getLinearVelocity();

        // Send state variables to the adaptive controller
        update_state_vals(position[0], velocity[0], position[1], velocity[1], position[2], velocity[2], angle[0], angle_rate[0], angle[1], angle_rate[1], angle[2], angle_rate[2]);

        update_goals();

        run();

        // Replace PID control variables with Adaptive control variables
        output_controls_[0] = get_U1();
        output_controls_[1] = get_U2();
        output_controls_[2] = get_U3();
        output_controls_[3] = get_U4();

        return output_controls_;
    }

    virtual bool isLastGoalModeAllPassthrough() override
    {
        is_last_goal_mode_all_passthrough_ = true;

        for (unsigned int axis = 0; axis < Axis4r::AxisCount(); ++axis) {
            if (last_mode_[axis] != GoalModeType::Passthrough) {
                is_last_goal_mode_all_passthrough_ = false;
            }
        }

        return is_last_goal_mode_all_passthrough_;
    }

private:
    const IBoardClock* clock_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;
    Axis4r output_controls_;

    //inertia parameters
    const double Ix = 0.02f;
    const double Iy = 0.01f;
    const double Iz = 0.03f;

    const double l = 0.11f; //arm length, can make more accurate by being specific about lx, ly
    const double m = 0.916f; // mass in kg
    const double grav = 9.81f; // gravity
    const double Jr = 0.00006f; // inertia of rotor, currently an estimate; make more accurate by getting a measured value

    // Static Gain Variables
    const double k_phi = -16.75f; // roll angle //16.75
    const double k_theta = -26.75f; //  pitch angle //26.75
    const double k_psi = -1.0f; //  yaw angle //13
    const double k_roll = -450.0f; // roll rate //450
    const double k_pitch = -450.0f; // pitch rate //450
    const double k_yaw = -40000.0f; // yaw rate //400

    //input saturation
    const double U1_sat = 1.0f;
    const double U2_sat = .95f;
    const double U3_sat = .95f;
    const double U4_sat = .95f;

    //trajectory parameters
    const double pi = 3.14159265359f;
    const double period = 45.0f;
    const double radius = 2.5f; // input radius of the circle
    const double alt = 5.0f; // height used for circle/square

    // other constants
    const double NEQN = 7.0f;

    bool reset = true;
    double x_0[12];
    GoalMode last_mode_;
    bool is_last_goal_mode_all_passthrough_;
    //double error[3] = { 0 };
    double ref_vec[10][3] = { { 0 } };
    double ref_sum[3] = { 0 };
    double velocity_integrator[3] = { 0 };
    static constexpr int array_length = 7;
    double zero[array_length] = { 0 };
    double* adaptive_y = zero;
    double* adaptive_output = zero;
    double last_yaw = 0.0f;

    //********************** SlidingModeModel Variables ******************************************/

    // state values
    double x_in, xdot_in, y_in, ydot_in, z_in, zdot_in, phi_in, P_in, theta_in, Q_in, psi_in, R_in;

    double x_des;
    double y_des;

    // State Vector
    double x[12][1];
    double reference[12] = { 0 };

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

    void update_state_vals(double x_val, double vx, double y_val, double vy, double z_val, double vz, double roll, double roll_rate, double pitch, double pitch_rate, double yaw, double yaw_rate)
    {
        x_in = x_val;
        xdot_in = vx;
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

    void update_goals()
    {
        const auto& mode = goal_->getGoalMode();
        const auto& value = goal_->getGoalValue();

        for (int i = 0; i < 12; i++) {
            reference[i] = 10001.0f;
        }
        int count1 = 0, count2 = 0, count3 = 0, count4 = 0;

        for (unsigned int axis = 0; axis < Axis4r::AxisCount(); ++axis) {
            switch (mode[axis]) {
            case GoalModeType::AngleRate:

                switch (count1) {
                case 0:
                    reference[11] = value[axis];
                    break;
                case 1:
                    reference[9] = value[axis];
                    break;
                case 2:
                    reference[10] = value[axis];
                    break;
                }
                count1++;
                break;
            case GoalModeType::AngleLevel:

                switch (count2) {
                case 0:
                    if (axis == 2) {
                        reference[8] = value[axis];
                    }
                    else {
                        reference[6] = value[axis];
                    }
                    break;
                case 1:
                    reference[7] = -value[axis];
                    break;
                case 2:
                    reference[8] = value[axis];
                    break;
                }
                count2++;
                break;
            case GoalModeType::VelocityWorld:

                switch (count3) {
                case 0:
                    reference[3] = value[axis];
                    break;
                case 1:
                    reference[4] = value[axis];
                    break;
                case 2:
                    reference[5] = value[axis];
                    break;
                }
                count3++;
                break;
            case GoalModeType::PositionWorld:

                switch (count4) {
                case 0:
                    if (axis != 0) {
                        reference[2] = value[axis];
                    }
                    else {
                        reference[0] = value[axis];
                    }
                    break;
                case 1:
                    reference[1] = value[axis];
                    break;
                case 2:
                    reference[2] = value[axis];
                    break;
                }
                count4++;
                break;
            default:

                break;
            }
            if (mode[axis] != last_mode_[axis]) {
                reset = true;
            }
            last_mode_[axis] = mode[axis];
        }
        if (reference[8] < 10000) {
            last_yaw = reference[8];
        }
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

    void Sliding_Iconfig(double zddot, double x_desired, double x_current, double xdot, double y_desired, double y_current, double ydot, double yaw_current, double result[2][1])
    {
        double Fz, xddot, yddot;
        int i;
        Fz = (zddot + 9.81) * m;
        xddot = -2.0 * xdot - 2.0 * (x_current - x_desired);
        yddot = -2.0 * ydot - 3.0 * (y_current - y_desired);
        if (Fz == 0) {
            Fz = 9.81 * m;
        }
        result[0][0] = (xddot * sin(yaw_current) - yddot * cos(yaw_current)) * m / Fz;
        result[1][0] = (xddot * cos(yaw_current) + yddot * sin(yaw_current)) * m / Fz;
        // Limit the angle reference values
        for (i = 0; i < 2; i++) {
            if (result[i][0] > 0.3) {
                result[i][0] = 0.3;
            }
            else if (result[i][0] < -0.3) {
                result[i][0] = -0.3;
            }
        }
    }

    void inverse_3x3(double A[3][3], double result[3][3])
    {
        double det_A; // dummy variable
        det_A = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) + A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * A[2][0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
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
            result[0][0] = (1 / det_A) * (A[1][1] * A[2][2] - A[1][2] * A[2][1]);
            result[0][1] = (1 / det_A) * (A[0][2] * A[2][1] - A[0][1] * A[2][2]);
            result[0][2] = (1 / det_A) * (A[0][1] * A[1][2] - A[0][2] * A[1][1]);
            result[1][0] = (1 / det_A) * (A[1][2] * A[2][0] - A[1][0] * A[2][2]);
            result[1][1] = (1 / det_A) * (A[0][0] * A[2][2] - A[0][2] * A[2][0]);
            result[1][2] = (1 / det_A) * (A[0][2] * A[1][0] - A[0][0] * A[1][2]);
            result[2][0] = (1 / det_A) * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
            result[2][1] = (1 / det_A) * (A[0][1] * A[2][0] - A[0][0] * A[2][1]);
            result[2][2] = (1 / det_A) * (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
        }
    }

    void PQR_generation(double states[12][1], double result[3][1])
    {
        result[0][0] = states[7][0] - sin(states[6][0]) * states[10][0];
        result[1][0] = states[9][0] * cos(states[6][0]) + states[10][0] * (cos(states[8][0]) * sin(states[6][0]));
        result[2][0] = states[11][0] * (cos(states[8][0]) * cos(states[6][0])) - states[9][0] * sin(states[6][0]);
    }

    void Angular_velocities_from_PQR(double PQR_val[3][1], double Angles_val[3][1], double result[3][1])
    {
        result[0][0] = PQR_val[0][0] + PQR_val[1][0] * sin(Angles_val[0][0]) * tan(Angles_val[1][0]) + PQR_val[2][0] * cos(Angles_val[0][0]) * tan(Angles_val[1][0]);
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

    void rungeKutta(double* y, double* yp, uint64_t t_val, double dt, int size)
    {
        double zero_vec[array_length] = { 0 };
        double k1[array_length] = { 0 };
        double k2[array_length] = { 0 };
        double k3[array_length] = { 0 };
        double k4[array_length] = { 0 };
        double* y_temp;
        double* y_out = zero_vec;
        y = yp;
        y_temp = y;
        model(y_temp, static_cast<double>(t_val), y_out);
        for (int n = 0; n < size; n++) {
            k1[n] = dt * y_out[n];
            y_temp[n] = y[n] + k1[n] / 2;
        }
        model(y_temp, t_val + dt / 2, y_out);
        for (int n = 0; n < size; n++) {
            k2[n] = dt * y_out[n];
            y_temp[n] = y[n] + k2[n] / 2;
        }
        model(y_temp, t_val + dt / 2, y_out);
        for (int n = 0; n < size; n++) {
            k3[n] = dt * y_out[n];
            y_temp[n] = y[n] + k3[n];
        }
        model(y_temp, t_val + dt / 2, y_out);
        for (int n = 0; n < size; n++) {
            k4[n] = dt * y_out[n];
            yp[n] = y[n] + k1[n] / 6 + k2[n] / 3 + k3[n] / 4 + k4[n] / 6;
        }
    }

    // ---------------------------------------------------------------------------
    // SlidingModeModel method
    // ---------------------------------------------------------------------------
    void model(double* y, double last_time_, double* y_out)
    {
        unused(last_time_);

        /********** Input the State Vector***********/
        x[0][0] = x_in;
        x[1][0] = xdot_in;
        x[2][0] = y_in;
        x[3][0] = ydot_in;
        x[4][0] = z_in;
        x[5][0] = zdot_in;
        x[6][0] = phi_in;
        PQR[0][0] = P_in;
        x[8][0] = theta_in;
        PQR[1][0] = Q_in;
        x[10][0] = -psi_in;
        PQR[2][0] = R_in;

        if (reset) {
            for (int i = 0; i < 12; i++) {
                x_0[i] = x[i][0];
            }
            velocity_integrator[2] = 0.0f;
            reset = false;
        }

        double velocity_real[3] = { x[1][0], x[3][0], x[5][0] };
        double velocity_goal[3] = { reference[4], reference[3], reference[5] };

        for (int i = 0; i < 3; i++) {
            ref_sum[i] = 0;
            for (int j = 0; j < 9; j++) {
                ref_vec[j][i] = ref_vec[j + 1][i];
            }
            if (abs(velocity_goal[i]) < 10000) {
                if (i == 0) {
                    velocity_integrator[i] += (velocity_real[i] - velocity_goal[i] - x_0[2 * i + 1]) * 0.004f;
                    ref_vec[9][i] = -x[2 * i][0] - velocity_integrator[i];
                }
                if (i == 1) {
                    velocity_integrator[i] += (velocity_real[i] - velocity_goal[i] - x_0[2 * i + 1]) * 0.0007f;
                    ref_vec[9][i] = x[2 * i][0] - velocity_integrator[i];
                }
                if (i == 2) {
                    velocity_integrator[i] += (-velocity_real[i] - velocity_goal[i] - x_0[2 * i + 1]) * 0.0005f;
                    ref_vec[9][i] = (x[2 * i][0] + velocity_integrator[i]);
                }
            }
            if (abs(velocity_goal[i]) > 10000) {
                ref_vec[9][i] = reference[i];
                if (i == 0) {
                    ref_vec[9][0] = reference[1];
                }
                else if (i == 1) {
                    ref_vec[9][1] = reference[0];
                }
                if (i == 2) {
                    ref_vec[9][i] = -ref_vec[9][i];
                }
                if (abs(ref_vec[9][i]) > 10000) {
                    ref_vec[9][i] = x[2 * i][0];
                    if (i == 0) {
                        ref_vec[9][i] = x[0][0];
                    }
                    if (i == 1) {
                        ref_vec[9][i] = x[2][0];
                    }
                }
            }

            if (i > 0) {
                ref_vec[9][i] = -ref_vec[9][i];
            }
            for (int j = 0; j < 10; j++) {
                ref_sum[i] += ref_vec[j][i];
            }
            ref_sum[i] = ref_sum[i] /= 10; //ref_vec[9][i];//
        }

        x_des = ref_sum[0];
        if (x_des > 10000) {
            x_des = x[0][0];
        }

        y_des = ref_sum[1];
        if (y_des > 10000) {
            y_des = x[2][0];
        }

        r[0][0] = -ref_sum[2];
        if (abs(r[0][0]) > 10000) {
            r[0][0] = x[4][0];
        }

        r[3][0] = reference[8];
        if (r[3][0] > 10000) {
            r[3][0] = -last_yaw;
        }

        double lambda_theta = 0.5f;
        double lambda_theta_rate = 0.5f;
        double lambda_phi = 0.95f;
        double lambda_phi_rate = 0.95f;
        double lambda_psi = 0.0004f;
        double lambda_psi_rate = 0.05f;
        double lambda_z = 0.5f;

        Angular_velocities_from_PQR(PQR, Angles, Angular_rates);
        x[7][0] = Angular_rates[0][0];
        x[9][0] = Angular_rates[1][0];
        x[11][0] = Angular_rates[2][0];

        // -9, -21 are adjustable gains for the z controller
        zdotdot = -9 * x[5][0] - 21 * (x[4][0] - r[0][0]);

        /**********Iconfig Sliding Mode***********************/

        Sliding_Iconfig(zdotdot, x_des, x[0][0], x[1][0], y_des, x[2][0], x[3][0], x[10][0], refs);
        ref_angles[0][0] = refs[0][0];
        ref_angles[1][0] = refs[1][0];

        /* Reference angles to be sent to inner loop */
        r[1][0] = ref_angles[0][0]; //phi ref
        if (reference[6] < 10000 && reference[6] != 0.0f) {
            r[1][0] = reference[6];
        }
        r[2][0] = ref_angles[1][0]; //theta ref
        if (reference[7] < 10000 && reference[7] != 0.0f) {
            r[2][0] = reference[7];
        }

        /************ Iconfig Control Law *********************/
        // First get integrated uncertainty parameters
        delta_z = y[6];
        delta_phi = y[0];
        delta_theta = y[1];
        delta_psi = y[2];
        delta_roll = y[3];
        delta_pitch = y[4];
        delta_yaw = y[5];

        U1 = (zdotdot + grav) * m + delta_z;

        y_out[6] = lambda_z * zdotdot; // generate sliding surface in z, .015 is adjustable slope for sliding surface

        // error in euler angles
        S2_phi = x[6][0] - r[1][0];
        S2_theta = x[8][0] - r[2][0];
        S2_psi = -x[10][0] - r[3][0];
        if (S2_psi > M_PI) {
            S2_psi -= 2 * M_PI;
        }
        if (S2_psi < -M_PI) {
            S2_psi += 2 * M_PI;
        }

        // generate delta_dot which goes to integrator variable, sliding surface for 3 euler angles, can adjust sliding surface slope as desired
        y_out[0] = lambda_phi * S2_phi;
        y_out[1] = lambda_theta * S2_theta;
        y_out[2] = lambda_psi * S2_psi;

        R_matrix[0][0] = 1;
        R_matrix[1][0] = sin(x[6][0] * tan(x[8][0]));
        R_matrix[2][0] = cos(x[6][0]) * tan(x[8][0]);
        R_matrix[1][0] = 0;
        R_matrix[1][1] = cos(x[6][0]);
        R_matrix[1][2] = -1 * sin(x[6][0]);
        R_matrix[2][0] = 0;
        R_matrix[2][1] = sin(x[6][0]) / cos(x[8][0]);
        R_matrix[2][2] = cos(x[6][0]) / cos(x[8][0]);

        inverse_3x3(R_matrix, R_inverse);
        rollrate_ref = R_inverse[0][0] * S2_phi * k_phi + R_inverse[0][1] * S2_theta * k_theta + R_inverse[0][2] * S2_psi * k_psi - delta_phi;
        pitchrate_ref = R_inverse[1][0] * S2_phi * k_phi + R_inverse[1][1] * S2_theta * k_theta + R_inverse[1][2] * S2_psi * k_psi - delta_theta;
        yawrate_ref = R_inverse[2][0] * S2_phi * k_phi + R_inverse[2][1] * S2_theta * k_theta + R_inverse[2][2] * S2_psi * k_psi - delta_psi;

        if (reference[9] < 10000 && reference[9] != 0.0f) {
            rollrate_ref = reference[9];
        }
        if (reference[10] < 10000 && reference[10] != 0.0f) {
            pitchrate_ref = reference[10];
        }
        if (reference[11] < 10000) {
            yawrate_ref = reference[11];
        }

        PQR_generation(x, PQR);
        S3_P = PQR[0][0] - rollrate_ref;
        S3_Q = PQR[1][0] - pitchrate_ref;
        S3_R = PQR[2][0] - yawrate_ref;

        // Sliding surface for the body frame angular rates to be integrated
        y_out[3] = lambda_phi_rate * S3_P;
        y_out[4] = lambda_theta_rate * S3_Q;
        y_out[5] = lambda_psi_rate * S3_R;

        // Calculate controls for roll, pitch, and yaw
        U2 = k_roll * S3_P * Ix + (Iz - Iy) * PQR[1][0] * PQR[2][0] - delta_roll;
        U3 = k_pitch * S3_Q * Iy + (Ix - Iz) * PQR[0][0] * PQR[2][0] - delta_pitch;
        U4 = k_yaw * S3_R * Iz + (Iy - Ix) * PQR[0][0] * PQR[1][0] - delta_yaw;

        // Rescale such that the outputs normalize from -1,1

        U1 = U1 / 80; //sqrt(abs(U1)) / 6.20; // I used sqrt to try and allow for smoother signal

        U2 = U2 / 80;

        U3 = U3 / 80;

        U4 = U4 / 80;

        // Saturations: U1->.35,1 : U2,U3,U4 -> -.2,.2
        if (U1 > 1) {
            U1 = 1;
        }
        else if (U1 < 0.35) {
            U1 = 0.35;
        }
        if (U2 > U2_sat) {
            U2 = U2_sat;
        }
        else if (U2 < -U2_sat) {
            U2 = -U2_sat;
        }
        if (U3 > U3_sat) {
            U3 = U3_sat;
        }
        else if (U3 < -U3_sat) {
            U3 = -U3_sat;
        }
        if (U4 > U4_sat) {
            U4 = U4_sat;
        }
        else if (U4 < -U4_sat) {
            U4 = -U4_sat;
        }

        remapU(U1, U2, U3, U4); //remap to axis4r

    } // SlidingModeModel */
};
}
#endif
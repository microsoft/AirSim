// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_multirotor_hpp
#define msr_airlib_multirotor_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "Rotor.hpp"
#include "controllers/ControllerBase.hpp"
#include "MultiRotorParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"


namespace msr { namespace airlib {

class MultiRotor : public PhysicsBody {
public:
    MultiRotor()
    {
        //allow default constructor with later call for initialize
    }
    MultiRotor(MultiRotorParams* params, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        initialize(params, initial_kinematic_state, environment);
    }
    void initialize(MultiRotorParams* params, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        params_ = params;

        PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, initial_kinematic_state, environment);

        createRotors(*params_, rotors_, environment);

        initSensors(*params_, getKinematics(), getEnvironment());

        //setup drag factors (must come after createRotors).
        setupDragFactors();

        MultiRotor::reset();
    }

    DroneControllerBase* getController()
    {
        return params_->getController();
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        //reset inputs
        if (getController())
            getController()->reset();

        //reset rotors, kinematics and environment
        PhysicsBody::reset();

        //update drag factors after environment and kinematics is reset
        updateDragFactors();

        //reset rotors after environment is reset
        for (Rotor& rotor : rotors_)
            rotor.reset();

        //reset sensors last after their ground truth has been reset
        resetSensors();
    }

    virtual void update() override
    {
        updateDragFactors();

        //update forces and environment as a result of last dt
        PhysicsBody::update();
    }
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        PhysicsBody::reportState(reporter);

        reportSensors(*params_, reporter);

        //report rotors
        for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
            reporter.startHeading("", 1);
            reporter.writeValue("Rotor", rotor_index);
            reporter.endHeading(false, 1);
            rotors_.at(rotor_index).reportState(reporter);
        }
    }
    //*** End: UpdatableState implementation ***//


    //implement abstract methods from PhysicsBody
    virtual void kinematicsUpdated() override
    {
        updateSensors(*params_, getKinematics(), getEnvironment());

        getController()->update();

        //transfer new input values from controller to rotors
        for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
            rotors_.at(rotor_index).setControlSignal(
                getController()->getVertexControlSignal(rotor_index));
        }
    }

    //sensor getter
    const SensorCollection& getSensors() const
    {
        return params_->getSensors();
    }

    //physics body abstract interface
    virtual Vector3r getLinearDragFactor() const override
    {
        return linear_drag_factor_;
    }
    virtual Vector3r getAngularDragFactor() const override
    {
        return angular_drag_factor_;
    }
    virtual uint vertexCount() const  override
    {
        return params_->getParams().rotor_count;
    }
    virtual PhysicsBodyVertex& getVertex(uint index)  override
    {
        return rotors_.at(index);
    }
    virtual const PhysicsBodyVertex& getVertex(uint index) const override
    {
        return rotors_.at(index);
    }
    virtual real_T getRestitution() const override
    {
        return params_->getParams().restitution;
    }
    virtual real_T getFriction()  const override
    {
        return params_->getParams().friction;
    }

    Rotor::Output getRotorOutput(uint rotor_index) const
    {
        return rotors_.at(rotor_index).getOutput();
    }

    virtual ~MultiRotor() = default;

private: //methods
    static void createRotors(const MultiRotorParams& params, vector<Rotor>& rotors, const Environment* environment)
    {
        rotors.clear();
        //for each rotor pose
        for (uint rotor_index = 0; rotor_index < params.getParams().rotor_poses.size(); ++rotor_index) {
            const MultiRotorParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
            rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
        }
    }

    void reportSensors(MultiRotorParams& params, StateReporter& reporter)
    {
        params.getSensors().reportState(reporter);
    }

    void updateSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
    {
        params.getSensors().update();
    }

    void initSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
    {
        params.getSensors().initialize(&state, &environment);
    }

    void resetSensors()
    {
        params_->getSensors().reset();
    }

    real_T getAngDragIntOverFaceXY(real_T x, real_T y, real_T z)
    {
        /* Integral: 2 * integral_0^(y/2) integral_0^(x/2) (3/4 (a^2 + b^2) z + z^3/8) da db = 1/64 x y z (x^2 + y^2 + 2 z^2)
        https://www.wolframalpha.com/input/?i=integral_0%5E(y%2F2)+integral_0%5E(x%2F2)+(3%2F4)*(a%5E2+%2B+b%5E2)*z+%2B+z%5E3%2F8+da+db
        we multiply by 2 for l=-x/2 to 0 and l = 0 to x/2
        */
        return 2.0f * 1.0f/64 * (x * y * z * (x*x + y*y + 2*z*z));
    }

    void setupDragFactors()
    {
        const auto& params = params_->getParams();

        /************* Linear drag *****************/
        //we use box as approximate size with dimensions x, y, z plus the area of propellers when they rotate
        //while moving along any axis, we find area that will be exposed in that direction
        real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;
        real_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;
        {
            real_T top_bottom_area = params.body_box.x * params.body_box.y;
            real_T left_right_area = params.body_box.x * params.body_box.z;
            real_T front_back_area = params.body_box.y * params.body_box.z;
            linear_drag_factor_unit_ = Vector3r(
                front_back_area + rotors_.size() * propeller_xsection, 
                left_right_area + rotors_.size() * propeller_xsection, 
                top_bottom_area) 
                * params.linear_drag_coefficient / 2; 
        }

        /************* Angular drag ******************************************************************
        Ref: http://physics.stackexchange.com/a/305020/14061
        We will ignore drag due to shear stress. Drag due to angular friction is modelled for 
        square body + cylinderical area of rotating propeller. For square box of dimension x,y,z
        consider each face, for example, x-y face. The drag torque T_d is generated due to velocity 
        vector at each point on body which produces dynamic pressure P_d.
        T_d = F_d X r = F_d * r (because both are perpendicular)
        F_d = C * integrate(P_d) over area
        P_d = 1/2 * rho * v^2
        v = r X w = r * w (because both are perpendicular)
        Thus we get (y is just width of face, torque summed over width),
        T_d = C * integrate(1/2 * rho * (r*w)^2 * r) * y
        T_d = 1/2 * rho * C * * w^2 * y * integrate(r^3) over length x
        Now r = sqrt(a^2 + b^2 + (z/2)^2) for a = -x/2 to x/2, b = -y/2 to y/2
        Now we have to integrate r^(3/2) which is difficult. So instead we do taylor expansion
        of (a^2 + b^2 + (z/2)^2)^3/2 and taking only first order terms we get
        (a^2 + b^2 + (z/2)^2)^(3/2) ~ (z^3) / 8 + (3/4) *(a^2 + b^2) * z
        Integral: 2 * integral_0^(y/2) integral_0^(x/2) (3/4 (a^2 + b^2) z + z^3/8) da db = 1/64 x y z (x^2 + y^2 + 2 z^2)
        https://www.wolframalpha.com/input/?i=integral_0%5E(y%2F2)+integral_0%5E(x%2F2)+(3%2F4)*(a%5E2+%2B+b%5E2)*z+%2B+z%5E3%2F8+da+db

        For propellers, we assume that arm length >> propeller radius so that we can simply multiply
        (omega * r)^2 * r with area or cross section of rotating propeller
        *********************************************************************************************/
        {
            real_T arm_length_cube_sum = 0.0f;
            for (uint i = 0; i < params.rotor_poses.size(); ++i)
                arm_length_cube_sum += pow(params.rotor_poses.at(i).position.norm(), 3.0f);

            real_T top_bottom_area = getAngDragIntOverFaceXY(params.body_box.x, params.body_box.y, params.body_box.z);
            real_T left_right_area = getAngDragIntOverFaceXY(params.body_box.z, params.body_box.x, params.body_box.y); 
            real_T front_back_area = getAngDragIntOverFaceXY(params.body_box.y, params.body_box.z, params.body_box.x); 
            angular_drag_factor_unit_ = Vector3r(
                2 * (top_bottom_area + front_back_area) + arm_length_cube_sum * propeller_area,
                2 * (top_bottom_area + left_right_area) + arm_length_cube_sum * propeller_area,
                2 * (left_right_area + front_back_area) + arm_length_cube_sum * propeller_xsection)
                * params.angular_drag_coefficient / 2;
        }
    }

    void updateDragFactors()
    {
        //update drag factors
        real_T air_density = hasEnvironment() ? getEnvironment().getState().air_density : EarthUtils::SeaLevelAirDensity;
        linear_drag_factor_ = linear_drag_factor_unit_ * air_density;
        angular_drag_factor_ = angular_drag_factor_unit_ * air_density;
    }


private: //fields
    MultiRotorParams* params_;

    //let us be the owner of rotors object
    vector<Rotor> rotors_;

    //drag
    Vector3r linear_drag_factor_unit_, angular_drag_factor_unit_;
    Vector3r linear_drag_factor_, angular_drag_factor_;
};

}} //namespace
#endif

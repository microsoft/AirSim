// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_multirotor_hpp
#define msr_air_copter_sim_multirotor_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "Rotor.hpp"
#include "controllers/ControllerBase.hpp"
#include "MultiRotorParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
//sensors
#include "sensors/imu/ImuSimple.hpp"
#include "sensors/magnetometer/MagnetometerSimple.hpp"
#include "sensors/barometer/BarometerSimple.hpp"
#include "sensors/gps/GpsSimple.hpp"

namespace msr { namespace airlib {

class MultiRotor : public PhysicsBody {
public:
    MultiRotor()
    {
        MultiRotor::reset();
    }
    MultiRotor(const MultiRotorParams& params, const Kinematics::State& initial_kinematic_state, Environment* environment, ControllerBase* controller_ptr)
    {
        initialize(params, initial_kinematic_state, environment, controller_ptr);
    }
	void initialize(const MultiRotorParams& params, const Kinematics::State& initial_kinematic_state, Environment* environment, ControllerBase* controller_ptr)
	{
		params_ = params;
        controller_ptr_ = controller_ptr;

        PhysicsBody::initialize(params_.mass, params_.inertia, initial_kinematic_state, environment);

        createRotors(params_, rotors_, environment);

        createSensors(params_, initial_kinematic_state, getEnvironment());

		//setup drag factors (must come after createRotors).
		setupDragFactors();

        MultiRotor::reset();
	}


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        //reset inputs
        if (controller_ptr_)
            controller_ptr_->reset();

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

    virtual void update(real_T dt) override
    {
        updateDragFactors();

        //update forces and environment as a result of last dt
        PhysicsBody::update(dt);
    }
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        PhysicsBody::reportState(reporter);

        reportSensors(params_, reporter);

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
    virtual void kinematicsUpdated(real_T dt) override
    {
        updateSensors(params_, getKinematics(), getEnvironment(), dt);

        controller_ptr_->update(dt);

        //transfer new input values from controller to rotors
        for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
            rotors_.at(rotor_index).setControlSignal(
                controller_ptr_->getVertexControlSignal(rotor_index));
        }
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
        return static_cast<uint>(params_.rotor_poses.size());
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
        return params_.restitution;
    }
    virtual real_T getFriction()  const override
    {
        return params_.friction;
    }

    //sensor getters
    const ImuBase* getImu() const 
    {
        return imu_.get();
    }
    const MagnetometerBase* getMagnetometer() const 
    {
        return magnetometer_.get();
    }
    const BarometerBase* getBarometer() const 
    {
        return barometer_.get();
    }
    const GpsBase* getGps() const 
    {
        return gps_.get();
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
        for (uint rotor_index = 0; rotor_index < params.rotor_poses.size(); ++rotor_index) {
            const MultiRotorParams::RotorPose& rotor_pose = params.rotor_poses.at(rotor_index);
            rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.rotor_params, environment, rotor_index);
        }
    }

    void reportSensors(const MultiRotorParams& params, StateReporter& reporter)
    {
        if (params.enabled_sensors.imu) {
            imu_->reportState(reporter);
        }
        if (params.enabled_sensors.magnetometer) {
            magnetometer_->reportState(reporter);
        }
        if (params.enabled_sensors.gps) {
            gps_->reportState(reporter);
        }
        if (params.enabled_sensors.barometer) {
            barometer_->reportState(reporter);
        }
    }

    void updateSensors(const MultiRotorParams& params, const Kinematics::State& state, const Environment& environment, real_T dt)
    {
        if (params.enabled_sensors.imu) {
            imu_->update(dt);
        }
        if (params.enabled_sensors.magnetometer) {
            magnetometer_->update(dt);
        }
        if (params.enabled_sensors.gps) {
            gps_->update(dt);
        }
        if (params.enabled_sensors.barometer) {
            barometer_->update(dt);
        }
    }

    void createSensors(const MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
    {
        sensors_ground_truth_.reset(new SensorBase::GroundTruth());
        sensors_ground_truth_->body = this;
        sensors_ground_truth_->environment = &getEnvironment();
        sensors_ground_truth_->kinematics = &getKinematics();

        //IMU
        if (params.enabled_sensors.imu) {
            imu_.reset(new ImuSimple(sensors_ground_truth_.get()));
        }

        //Magnetometer
        if (params_.enabled_sensors.magnetometer) {
            magnetometer_.reset(new MagnetometerSimple(sensors_ground_truth_.get()));
        }

        //GPS
        if (params_.enabled_sensors.gps) {
            gps_.reset(new GpsSimple(sensors_ground_truth_.get()));
        }

        //Barometer
        if (params_.enabled_sensors.barometer) {
            barometer_.reset(new BarometerSimple(sensors_ground_truth_.get()));
        }
    }

    void resetSensors()
    {
        if (imu_)
            imu_->reset();
        if (magnetometer_)
            magnetometer_->reset();
        if (gps_)
            gps_->reset();
        if (barometer_)
            barometer_->reset();
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
        /************* Linear drag *****************/
        //we use box as approximate size with dimensions x, y, z plus the area of propellers when they rotate
        //while moving along any axis, we find area that will be exposed in that direction
        real_T propeller_area = M_PIf * params_.rotor_params.propeller_radius * params_.rotor_params.propeller_radius;
        real_T propeller_xsection = M_PIf * params_.rotor_params.propeller_radius * params_.rotor_params.propeller_height;
        {
            real_T top_bottom_area = params_.dim.x * params_.dim.y;
            real_T left_right_area = params_.dim.x * params_.dim.z;
            real_T front_back_area = params_.dim.y * params_.dim.z;
            linear_drag_factor_unit_ = Vector3r(
                front_back_area + rotors_.size() * propeller_xsection, 
                left_right_area + rotors_.size() * propeller_xsection, 
                top_bottom_area) 
                * params_.linear_drag_coefficient / 2; 
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
            for (uint i = 0; i < params_.rotor_poses.size(); ++i)
                arm_length_cube_sum += pow(params_.rotor_poses.at(i).position.norm(), 3.0f);

            real_T top_bottom_area = getAngDragIntOverFaceXY(params_.dim.x, params_.dim.y, params_.dim.z);
            real_T left_right_area = getAngDragIntOverFaceXY(params_.dim.z, params_.dim.x, params_.dim.y); 
            real_T front_back_area = getAngDragIntOverFaceXY(params_.dim.y, params_.dim.z, params_.dim.x); 
            angular_drag_factor_unit_ = Vector3r(
                2 * (top_bottom_area + front_back_area) + arm_length_cube_sum * propeller_area,
                2 * (top_bottom_area + left_right_area) + arm_length_cube_sum * propeller_area,
                2 * (left_right_area + front_back_area) + arm_length_cube_sum * propeller_xsection)
                * params_.angular_drag_coefficient / 2;
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
    MultiRotorParams params_;
    ControllerBase* controller_ptr_ = nullptr;

    //let us be the owner of rotors object
    vector<Rotor> rotors_;

    //drag
    Vector3r linear_drag_factor_unit_, angular_drag_factor_unit_;
    Vector3r linear_drag_factor_, angular_drag_factor_;
    
    //sensors
    unique_ptr<SensorBase::GroundTruth> sensors_ground_truth_;
    unique_ptr<ImuBase> imu_;
    unique_ptr<MagnetometerBase> magnetometer_;
    unique_ptr<GpsBase> gps_;
    unique_ptr<BarometerBase> barometer_;
};

}} //namespace
#endif

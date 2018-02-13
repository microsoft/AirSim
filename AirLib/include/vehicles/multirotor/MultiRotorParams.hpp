// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MultiRotorParameters_hpp
#define msr_airlib_MultiRotorParameters_hpp

#include "common/Common.hpp"
#include "RotorParams.hpp"
#include "sensors/SensorCollection.hpp"
#include "controllers/DroneControllerBase.hpp"



namespace msr { namespace airlib {

class MultiRotorParams {
//All units are SI
public: //types
    struct RotorPose {
        Vector3r position;  //relative to center of gravity of vehicle body
        Vector3r normal;
        RotorTurningDirection direction;

        RotorPose()
        {}
        RotorPose(const Vector3r& position_val, const Vector3r& normal_val, RotorTurningDirection direction_val)
            : position(position_val), normal(normal_val), direction(direction_val)
        {}
    };

    struct EnabledSensors {
        bool imu = true;
        bool magnetometer = true;
        bool gps = true;
        bool barometer = true;
        bool distance = false; //this causes ray casts so disabled by default
    };

    struct Params {
        /*********** required parameters ***********/
        uint rotor_count;
        vector<RotorPose> rotor_poses;
        real_T mass;
        Matrix3x3r inertia;
        Vector3r body_box;

        /*********** optional parameters with defaults ***********/
        real_T linear_drag_coefficient = 1.3f / 4.0f; 
        //sample value 1.3 from http://klsin.bpmsg.com/how-fast-can-a-quadcopter-fly/, but divided by 4 to account
        // for nice streamlined frame design and allow higher top speed which is more fun.
        //angular coefficient is usually 10X smaller than linear, however we should replace this with exact number
        //http://physics.stackexchange.com/q/304742/14061
        real_T angular_drag_coefficient = linear_drag_coefficient; 
        real_T restitution = 0.15f;
        real_T friction = 0.7f;
        EnabledSensors enabled_sensors;
        RotorParams rotor_params;

        uint16_t api_server_port;
    };

public: //interface
    virtual void initialize()
    {
        sensor_storage_.clear();
        sensors_.clear();

        setupParams();

        addEnabledSensors(params_.enabled_sensors);
        controller_ = createController();
    }

    const Params& getParams() const
    {
        return params_;
    }
    Params& getParams()
    {
        return params_;
    }
    SensorCollection& getSensors()
    {
        return sensors_;
    }
    const SensorCollection& getSensors() const
    {
        return sensors_;
    }
    DroneControllerBase* getController()
    {
        return controller_.get();
    }
    const DroneControllerBase* getController() const
    {
        return controller_.get();
    }

    void addEnabledSensors(const EnabledSensors& enabled_sensors)
    {
        if (enabled_sensors.imu)
            addSensor(SensorBase::SensorType::Imu);
        if (enabled_sensors.magnetometer)
            addSensor(SensorBase::SensorType::Magnetometer);
        if (enabled_sensors.gps)
            addSensor(SensorBase::SensorType::Gps);
        if (enabled_sensors.barometer)
            addSensor(SensorBase::SensorType::Barometer);
        if (enabled_sensors.distance)
            addSensor(SensorBase::SensorType::Distance);
    }

    SensorBase* addSensor(SensorBase::SensorType sensor_type)
    {
        std::unique_ptr<SensorBase> sensor = createSensor(sensor_type);
        if (sensor) {
            SensorBase* sensor_temp = sensor.get();
            sensor_storage_.push_back(std::move(sensor));
            sensors_.insert(sensor_temp, sensor_type);
            return sensor_temp;
        }
        return nullptr;
    }


    virtual ~MultiRotorParams() = default;

protected: //must override by derived class
    virtual void setupParams() = 0;
    virtual std::unique_ptr<SensorBase> createSensor(SensorBase::SensorType sensor_type) = 0;
    virtual std::unique_ptr<DroneControllerBase> createController() = 0;

protected: //static utility functions for derived classes to use

    /// Initializes 4 rotors in the usual QuadX pattern:  http://ardupilot.org/copter/_images/MOTORS_QuadX_QuadPlus.jpg
    /// which shows that given an array of 4 motors, the first is placed top right and flies counter clockwise (CCW) and
    /// the second is placed bottom left, and also flies CCW.  The third in the array is placed top left and flies clockwise (CW)
    /// while the last is placed bottom right and also flies clockwise.  This is how the 4 items in the arm_lengths and
    /// arm_angles arrays will be used.  So arm_lengths is 4 numbers (in meters) where four arm lengths, 0 is top right, 
    /// 1 is bottom left, 2 is top left and 3 is bottom right.  arm_angles is 4 numbers (in degrees)  relative to forward vector (0,1), 
    /// provided in the same order where 0 is top right, 1 is bottom left, 2 is top left and 3 is bottom right, so for example, 
    /// the angles for a regular symmetric X pattern would be 45, 225, 315, 135.  The rotor_z is the offset of each motor upwards
    /// relative to the center of mass (in meters). 
    static void initializeRotorQuadX(vector<RotorPose>& rotor_poses /* the result we are building */,
        uint rotor_count /* must be 4 */, 
        real_T arm_lengths[], 
        real_T rotor_z /* z relative to center of gravity */)
    {
        Vector3r unit_z(0, 0, -1);  //NED frame
        if (rotor_count == 4) {
            rotor_poses.clear();

            /* Note: rotor_poses are built in this order:              
                 x-axis
            (2)  |   (0)
                 |
            -------------- y-axis
                 |
            (1)  |   (3)		
            */
            // vectors below are rotated according to NED left hand rule (so the vectors are rotated counter clockwise).
            Quaternionr quadx_rot(AngleAxisr(M_PIf / 4, unit_z));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[0], rotor_z), quadx_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[1], rotor_z), quadx_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[2], 0, rotor_z), quadx_rot, true), 
                unit_z, RotorTurningDirection::RotorTurningDirectionCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(-arm_lengths[3], 0, rotor_z), quadx_rot, true), 
                unit_z, RotorTurningDirection::RotorTurningDirectionCW);
        }
        else
            throw std::invalid_argument("Rotor count other than 4 is not supported by this method!");
    }

    static void initializeRotorHexX(vector<RotorPose>& rotor_poses /* the result we are building */,
        uint rotor_count /* must be 6 */,
        real_T arm_lengths[],
        real_T rotor_z /* z relative to center of gravity */)
    {
        Vector3r unit_z(0, 0, -1);  //NED frame
        if (rotor_count == 6) {
            rotor_poses.clear();
            /* Note: rotor_poses are built in this order: rotor 0 is CW
              See HEXA X configuration on http://ardupilot.org/copter/docs/connect-escs-and-motors.html

                     x-axis
                (2)    (4)
                   \  /
                    \/
               (1)-------(0) y-axis
                    /\                
                   /  \ 
                 (5)  (3)

            */

            // vectors below are rotated according to NED left hand rule (so the vectors are rotated counter clockwise).
            Quaternionr quadx_rot(AngleAxisr(M_PIf / 6, unit_z));
            Quaternionr no_rot(AngleAxisr(0, unit_z));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[0], rotor_z), no_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[1], rotor_z), no_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(arm_lengths[2], 0, rotor_z), quadx_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(-arm_lengths[3], 0, rotor_z), quadx_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[4], rotor_z), quadx_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCCW);
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, -arm_lengths[5], rotor_z), quadx_rot, true),
                unit_z, RotorTurningDirection::RotorTurningDirectionCW);
        }
        else
            throw std::invalid_argument("Rotor count other than 6 is not supported by this method!");
    }

    /// Initialize the rotor_poses given the rotor_count, the arm lengths and the arm angles (relative to forwards vector).
    /// Also provide the direction you want to spin each rotor and the z-offsetof the rotors relative to the center of gravity.
    static void initializeRotors(vector<RotorPose>& rotor_poses, uint rotor_count, real_T arm_lengths[], real_T arm_angles[], RotorTurningDirection rotor_directions[], real_T rotor_z /* z relative to center of gravity */)
    {
        Vector3r unit_z(0, 0, -1);  //NED frame
        rotor_poses.clear();
        for (uint i = 0; i < rotor_count; i++)
        {
            Quaternionr angle(AngleAxisr(arm_angles[i] * M_PIf / 180, unit_z));
            rotor_poses.emplace_back(VectorMath::rotateVector(Vector3r(0, arm_lengths[i], rotor_z), angle, true),
                unit_z, rotor_directions[i]);
        }
    }

    static void computeInertiaMatrix(Matrix3x3r& inertia, const Vector3r& body_box, const vector<RotorPose>& rotor_poses,
        real_T box_mass, real_T motor_assembly_weight)
    {
        inertia = Matrix3x3r::Zero();

        //http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html
        inertia(0, 0) = box_mass / 12.0f * (body_box.y()*body_box.y() + body_box.z()*body_box.z()); 
        inertia(1, 1) = box_mass / 12.0f * (body_box.x()*body_box.x() + body_box.z()*body_box.z()); 
        inertia(2, 2) = box_mass / 12.0f * (body_box.x()*body_box.x() + body_box.y()*body_box.y()); 

        for (size_t i = 0; i < rotor_poses.size(); ++i) {
            const auto& pos = rotor_poses.at(i).position;
            inertia(0, 0) += (pos.y()*pos.y() + pos.z()*pos.z()) * motor_assembly_weight;
            inertia(1, 1) += (pos.x()*pos.x() + pos.z()*pos.z()) * motor_assembly_weight;
            inertia(2, 2) += (pos.x()*pos.x() + pos.y()*pos.y()) * motor_assembly_weight;
        }
    }

private:
    Params params_;
    SensorCollection sensors_; //maintains sensor type indexed collection of sensors
    vector<unique_ptr<SensorBase>> sensor_storage_; //RAII for created sensors
    std::unique_ptr<DroneControllerBase> controller_;
};

}} //namespace
#endif

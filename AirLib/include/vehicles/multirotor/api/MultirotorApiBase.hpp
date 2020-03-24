// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DroneControlServer_hpp
#define air_DroneControlServer_hpp

#include "common/Common.hpp"
#include "MultirotorCommon.hpp"
#include "safety/SafetyEval.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "api/VehicleApiBase.hpp"

#include <atomic>
#include <thread>
#include <memory>

using namespace msr::airlib;

namespace msr { namespace airlib {

class MultirotorApiBase : public VehicleApiBase {

protected: //must be implemented

    /************************* low level move APIs *********************************/
    virtual void commandMotorPWMs(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm) = 0;
    virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) = 0;
    virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) = 0;
    virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) = 0;
    virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) = 0;
    virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) = 0;
    virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) = 0;
    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) = 0;
    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) = 0;
    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) = 0;

    /************************* set Controller Gains APIs *********************************/
    virtual void setControllerGains(uint8_t controllerType, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) = 0;

    /************************* State APIs *********************************/
    virtual Kinematics::State getKinematicsEstimated() const = 0;
    virtual LandedState getLandedState() const = 0;
    virtual GeoPoint getGpsLocation() const = 0;
    virtual const MultirotorApiParams& getMultirotorApiParams() const = 0;

    /************************* basic config APIs *********************************/
    virtual float getCommandPeriod() const = 0; //time between two command required for drone in seconds
    virtual float getTakeoffZ() const = 0;  // the height above ground for the drone after successful takeoff (Z above ground is negative due to NED coordinate system).
    //noise in difference of two position coordinates. This is not GPS or position accuracy which can be very low such as 1m.
    //the difference between two position cancels out transitional errors. Typically this would be 0.1m or lower.
    virtual float getDistanceAccuracy() const = 0; 

protected: //optional overrides but recommended, default values may work
    virtual float getAutoLookahead(float velocity, float adaptive_lookahead,
        float max_factor = 40, float min_factor = 30) const;
    virtual float getObsAvoidanceVelocity(float risk_dist, float max_obs_avoidance_vel) const;

    //below methods gets called by default implementations of move-related commands that would use a long 
    //running loop. These can be used by derived classes to do some init/cleanup.
    virtual void beforeTask()
    {
        //default is do nothing
    }
    virtual void afterTask()
    {
        //default is do nothing
    }

public: //optional overrides
    virtual void moveByRC(const RCData& rc_data);

    //below method exist for any firmwares that may want to use ground truth for debugging purposes
    virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment)
    {
        unused(kinematics);
        unused(environment);
    }

    virtual void resetImplementation() override;


public: //these APIs uses above low level APIs
    virtual ~MultirotorApiBase() = default;

    /************************* high level move APIs *********************************/
    //return value of these function is true if command was completed without interruption or timeouts
    virtual bool takeoff(float timeout_sec);
    virtual bool land(float timeout_sec);
    virtual bool goHome(float timeout_sec);

    virtual bool moveByMotorPWMs(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm, float duration);
    virtual bool moveByRollPitchYawZ(float roll, float pitch, float yaw, float z, float duration);
    virtual bool moveByRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle, float duration);
    virtual bool moveByRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle, float duration);
    virtual bool moveByRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z, float duration);
    virtual bool moveByAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z, float duration);
    virtual bool moveByAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle, float duration);
    virtual bool moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode);
    virtual bool moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode);
    virtual bool moveOnPath(const vector<Vector3r>& path, float velocity, float timeout_sec, DrivetrainType drivetrain, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead);
    virtual bool moveToPosition(float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain,
        const YawMode& yaw_mode, float lookahead, float adaptive_lookahead);
    virtual bool moveToZ(float z, float velocity, float timeout_sec, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead);
    virtual bool moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode);
    virtual bool rotateToYaw(float yaw, float timeout_sec, float margin);
    virtual bool rotateByYawRate(float yaw_rate, float duration);
    virtual bool hover();
    virtual RCData estimateRCTrims(float trimduration = 1, float minCountForTrim = 10, float maxTrim = 100);
    
    /************************* set angle gain APIs *********************************/
    virtual void setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd); 
    virtual void setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd); 
    virtual void setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd); 
    virtual void setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd); 

    /************************* Safety APIs *********************************/
    virtual void setSafetyEval(const shared_ptr<SafetyEval> safety_eval_ptr);
    virtual bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);

    /************************* high level status APIs *********************************/
    MultirotorState getMultirotorState() const
    {
        MultirotorState state;
        state.kinematics_estimated = getKinematicsEstimated();
        //TODO: add GPS health, accuracy in API
        state.gps_location = getGpsLocation();
        state.timestamp = clock()->nowNanos();
        state.landed_state = getLandedState();
        state.rc_data = getRCData();
        state.ready = isReady(state.ready_message);
        state.can_arm = canArm();
        return state;
    }

    /******************* Task management Apis ********************/
    virtual void cancelLastTask() override
    {
        token_.cancel();
    }

protected: //utility methods
    typedef std::function<bool()> WaitFunction;

    //*********************************safe wrapper around low level commands***************************************************
    virtual void moveByRollPitchYawZInternal(float roll, float pitch, float yaw, float z);
    virtual void moveByRollPitchYawThrottleInternal(float roll, float pitch, float yaw, float throttle);
    virtual void moveByRollPitchYawrateThrottleInternal(float roll, float pitch, float yaw_rate, float throttle);
    virtual void moveByRollPitchYawrateZInternal(float roll, float pitch, float yaw_rate, float z);
    virtual void moveByAngleRatesZInternal(float roll_rate, float pitch_rate, float yaw_rate, float z);
    virtual void moveByAngleRatesThrottleInternal(float roll_rate, float pitch_rate, float yaw_rate, float throttle);
    virtual void moveByVelocityInternal(float vx, float vy, float vz, const YawMode& yaw_mode);
    virtual void moveByVelocityZInternal(float vx, float vy, float z, const YawMode& yaw_mode);
    virtual void moveToPositionInternal(const Vector3r& dest, const YawMode& yaw_mode);

    /************* safety checks & emergency maneuvers ************/
    virtual bool emergencyManeuverIfUnsafe(const SafetyEval::EvalResult& result);
    virtual bool safetyCheckVelocity(const Vector3r& velocity);
    virtual bool safetyCheckVelocityZ(float vx, float vy, float z);
    virtual bool safetyCheckDestination(const Vector3r& dest_loc);

    /************* wait helpers ************/
    // helper function can wait for anything (as defined by the given function) up to the max_wait duration (in seconds).
    // returns true if the wait function succeeded, or false if timeout occurred or the timeout is invalid.
    Waiter waitForFunction(WaitFunction function, float max_wait);

    //useful for derived class to check after takeoff
    bool waitForZ(float timeout_sec, float z, float margin);

    /************* other short hands ************/
    virtual Vector3r getPosition() const
    {
        return getKinematicsEstimated().pose.position;
    }
    virtual Vector3r getVelocity() const
    {
        return getKinematicsEstimated().twist.linear;
    }
    virtual Quaternionr getOrientation() const
    {
        return getKinematicsEstimated().pose.orientation;
    }

    CancelToken& getCancelToken()
    {
        return token_;
    }

public: //types
    class UnsafeMoveException : public VehicleMoveException {
    public:
        const SafetyEval::EvalResult result;

        UnsafeMoveException(const SafetyEval::EvalResult result_val, const std::string& message = "")
            : VehicleMoveException(message), result(result_val)
        {}
    };

protected: //types
    class SingleCall {
    public:
        SingleCall(MultirotorApiBase* api)
            : api_(api) {
            auto& token = api->getCancelToken();

            //if we can't get lock, cancel previous call
            if (!token.try_lock()) {
                //TODO: should we worry about spurious failures in try_lock?
                token.cancel();
                token.lock();
            }

            if (isRootCall())
                token.reset();
            //else this is not the start of the call
        }

        virtual ~SingleCall()
        {
            auto& token = api_->getCancelToken();

            if (isRootCall())
                token.reset();
            //else this is not the end of the call

            token.unlock();
        }
    protected:
        MultirotorApiBase * getVehicleApi()
        {
            return api_;
        }

        bool isRootCall()
        {
            return api_->getCancelToken().getRecursionCount() == 1;
        }

    private:
        MultirotorApiBase* api_;
    };

    class SingleTaskCall : public SingleCall
    {
    public:
        SingleTaskCall(MultirotorApiBase* api)
            : SingleCall(api)
        {
            if (isRootCall())
                api->beforeTask();
        }

        virtual ~SingleTaskCall()
        {
            if (isRootCall())
                getVehicleApi()->afterTask();
        }
    };

    //use this lock for vehicle status APIs
    struct StatusLock {
        //this const correctness gymnastic is required because most
        //status update APIs are const
        StatusLock(const MultirotorApiBase* api)
            : lock_(
                * const_cast<std::recursive_mutex*>(& api->status_mutex_)
            )
        {
        }

    private:
        //we need mutable here because status APIs are const and shouldn't change data members
        mutable std::lock_guard<std::recursive_mutex> lock_;
    };

private: //types
    struct PathPosition {
        uint seg_index;
        float offset;
        Vector3r position;
    };

    struct PathSegment {
        Vector3r seg_normalized;
        Vector3r seg;
        float seg_length;
        float seg_velocity;
        float start_z;
        float seg_path_length;

        PathSegment(const Vector3r& start, const Vector3r& end, float velocity, float path_length)
        {
            seg = end - start;
            seg_length = seg.norm();
            seg_normalized = seg.normalized();
            start_z = start.z();
            seg_path_length = path_length;

            seg_velocity = velocity;
        }
    };

    //RAII
    class ObsStrategyChanger {
    private:
        shared_ptr<SafetyEval> safety_eval_ptr_;
        SafetyEval::ObsAvoidanceStrategy old_strategy_;
    public:
        ObsStrategyChanger(shared_ptr<SafetyEval> safety_eval_ptr, SafetyEval::ObsAvoidanceStrategy new_startegy)
        {
            safety_eval_ptr_ = safety_eval_ptr;
            old_strategy_ = safety_eval_ptr_->getObsAvoidanceStrategy();
            safety_eval_ptr_->setObsAvoidanceStrategy(new_startegy);
        }
        ~ObsStrategyChanger()
        {
            safety_eval_ptr_->setObsAvoidanceStrategy(old_strategy_);   
        }
    };

private: //methods
    float setNextPathPosition(const vector<Vector3r>& path, const vector<PathSegment>& path_segs,
        const PathPosition& cur_path_loc, float next_dist, PathPosition& next_path_loc);
    void adjustYaw(const Vector3r& heading, DrivetrainType drivetrain, YawMode& yaw_mode);
    void adjustYaw(float x, float y, DrivetrainType drivetrain, YawMode& yaw_mode);
    void moveToPathPosition(const Vector3r& dest, float velocity, DrivetrainType drivetrain, /* pass by value */ YawMode yaw_mode, float last_z);
    bool isYawWithinMargin(float yaw_target, float margin) const;

private: //variables
    CancelToken token_;
    std::recursive_mutex status_mutex_;
    RCData rc_data_trims_;
    shared_ptr<SafetyEval> safety_eval_ptr_;
    float obs_avoidance_vel_ = 0.5f;

    //TODO: make this configurable?
    float landing_vel_ = 0.2f; //velocity to use for landing
    float approx_zero_vel_ = 0.05f;
};

}} //namespace
#endif

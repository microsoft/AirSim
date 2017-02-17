// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DroneControlBase_hpp
#define air_DroneControlBase_hpp


#include "common/Common.hpp"
#include "Waiter.hpp"
#include "control/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "DroneControlCommon.hpp"

namespace msr { namespace airlib {

//This interface represents generic drone commands
//RETURN values: true if not prempted else false. For error conditions, raise exceptions.
class DroneControlBase {
public: //types
    class UnsafeMoveException : public MoveException {
    public:
        const SafetyEval::EvalResult result;

        UnsafeMoveException(const SafetyEval::EvalResult result_val, const string message = "")
            : MoveException(message), result(result_val)
        {}
    };

    struct StatusLock {
        StatusLock(DroneControlBase* drone)
            : drone_(drone), lock_(drone->status_mutex_)
        {
        }

    private:
        std::lock_guard<std::recursive_mutex> lock_;
        DroneControlBase* drone_;
    };

public: //interface for outside world
    
    enum class ImageType : uint {
        None = 0,
        Scene = 1, 
        Depth = 2, 
        Segmentation = 4,
        All = 255
    };
    typedef common_utils::EnumFlags<ImageType>  ImageTypeFlags;

    //atomic actions
    virtual bool armDisarm(bool arm, CancelableActionBase& cancelable_action) = 0;
    virtual bool requestControl(CancelableActionBase& cancelable_action) = 0;
    virtual bool releaseControl(CancelableActionBase& cancelable_action) = 0;
    virtual bool takeoff(float max_wait_seconds, CancelableActionBase& cancelable_action) = 0;
    virtual bool land(CancelableActionBase& cancelable_action) = 0;
    virtual bool goHome(CancelableActionBase& cancelable_action) = 0;

    /** \brief Move drone by specifieng roll, pitch angles in degrees, z in meters (NED) and yaw in degree.
    * \param cancelable_action This object implements preemptible sleep and allows to specify when is it safe to preempt 
    * \return true if completed without preempting else false
    * 
    * This method by itself will block until it finishes the execution OR cancelable_action signals for preempt.
    * This method can be called from ROS action server that runs on separate thread for non-blocking implementation. 
    */
    virtual bool moveByAngle(float pitch, float roll, float z, float yaw, float duration
        , CancelableActionBase& cancelable_action);

    /**
    * \brief Move drone by specifieng velocty components in 3 axis wrt to ground for given amount of time
    */
    virtual bool moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode,
        CancelableActionBase& cancelable_action);

    /**
    * \brief Move drone by specifieng velocty components in X-Y plan wrt to ground while maintaining hight
    */
    virtual bool moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode,
        CancelableActionBase& cancelable_action);
    virtual bool moveOnPath(const vector<Vector3r>& path, float velocity, DrivetrainType drivetrain, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead, CancelableActionBase& cancelable_action);

    /** \brief Move drone to position specified in NEU local coordinate system.
    */
    virtual bool moveToPosition(float x, float y, float z, float velocity, DrivetrainType drivetrain,
        const YawMode& yaw_mode, float lookahead, float adaptive_lookahead, CancelableActionBase& cancelable_action);
    virtual bool moveToZ(float z, float velocity, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead, CancelableActionBase& cancelable_action);
    virtual bool moveByManual(float vx_max, float vy_max, float z_min, DrivetrainType drivetrain, const YawMode& yaw_mode, float duration, CancelableActionBase& cancelable_action);
    virtual bool rotateToYaw(float yaw, float margin, CancelableActionBase& cancelable_action);
    virtual bool rotateByYawRate(float yaw_rate, float duration, CancelableActionBase& cancelable_action);
    virtual bool hover(CancelableActionBase& cancelable_action);

    //status getters
    virtual Vector3r getPosition() = 0;
    Vector2r getPositionXY();
    float getZ();
    virtual Vector3r getVelocity() = 0;
    virtual Quaternionr getOrientation() = 0;
    virtual RCData getRCData() = 0;
    virtual double timestampNow() = 0;
    virtual GeoPoint getHomePoint() = 0;
    virtual GeoPoint getGpsLocation() = 0;
    virtual bool isOffboardMode() = 0;

    virtual void setSafetyEval(const shared_ptr<SafetyEval> safety_eval_ptr);
    virtual bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);
    virtual const VehicleParams& getVehicleParams() = 0;

    //request image
    virtual bool setImageTypeForCamera(int camera_id, ImageType type);
    virtual ImageType getImageTypeForCamera(int camera_id);
    //get/set image
    virtual bool setImageForCamera(int camera_id, ImageType type, const vector<uint8_t>& image);
    virtual vector<uint8_t> getImageForCamera(int camera_id, ImageType type);


    DroneControlBase() = default;
    virtual ~DroneControlBase() = default;

protected: //types
    typedef std::function<bool()> WaitFunction;

protected: //must implement interface by derived class
    //low level commands
    //all angles in degrees, lengths in meters, velocities in m/s, durations in seconds
    //all coordinates systems are world NED (+x is North, +y is East, +z is down)
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) = 0;
    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) = 0;
    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) = 0;
    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) = 0;
    virtual void commandVirtualRC(const RCData& rc_data) = 0;
    virtual void commandEnableVirtualRC(bool enable) = 0;

    //config commands
    virtual float getCommandPeriod() = 0; //time between two command required for drone in seconds
    virtual float getTakeoffZ() = 0;  // the height above ground for the drone after successful takeoff (Z above ground is negative due to NED coordinate system).
                                      //noise in difference of two position coordinates. This is not GPS or position accuracy which can be very low such as 1m.
                                      //the difference between two position cancels out transitional errors. Typically this would be 0.1m or lower.
    virtual float getDistanceAccuracy() = 0; 

    //naked variables for derived class access
    unordered_map<int, ImageType> enabled_images;
    unordered_map<int, unordered_map<ImageType, vector<uint8_t>>> images;

protected: //optional oveerides recommanded for any drones, default implementation may work
    virtual float getAutoLookahead(float velocity, float adaptive_lookahead,
        float max_factor = 40, float min_factor = 30);
    virtual float getObsAvoidanceVelocity(float risk_dist, float max_obs_avoidance_vel);

protected: //higher level functions with no need to override in general
           //*********************************safe wrapper around low level commands***************************************************
    virtual bool moveByVelocity(float vx, float vy, float vz, const YawMode& yaw_mode);
    virtual bool moveByVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode);
    virtual bool moveToPosition(const Vector3r& dest, const YawMode& yaw_mode);
    virtual bool moveByRollPitchZ(float pitch, float roll, float z, float yaw);
    //****************************************************************************************************************************

protected: //utility functions and data members for derived classes

    // helper function can wait for anything (as defined by the given function) up to the max_wait duration (in seconds).
    // returns true if the wait function succeeded, or false if timeout occurred or the timeout is invalid.
    virtual bool waitForFunction(WaitFunction function, float max_wait, CancelableActionBase& cancelable_action);

    //useful for derived class to check after takeoff
    virtual bool waitForZ(float max_wait_seconds, float z, float margin, CancelableActionBase& cancelable_action);

    /************* safety checks & emergency manuevers ************/
    virtual bool emergencyManeuverIfUnsafe(const SafetyEval::EvalResult& result);
    virtual bool safetyCheckVelocity(const Vector3r& velocity);
    virtual bool safetyCheckVelocityZ(float vx, float vy, float z);
    virtual bool safetyCheckDestination(const Vector3r& dest_loc);
    void logHomePoint();

private:    //types
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

    //instances of this class is always local variable in DroneControlBase methods
    class VirtualRCEnable {
    private:
        DroneControlBase* drone_base_ptr_;
    public:
        VirtualRCEnable(DroneControlBase* drone_base_ptr)
        {
            drone_base_ptr_ = drone_base_ptr;
            drone_base_ptr_->commandEnableVirtualRC(true);
        }
        ~VirtualRCEnable()
        {
            drone_base_ptr_->commandEnableVirtualRC(false);
            //no need to worry about drone_base_ptr_
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

    bool isYawWithinMargin(float yaw_target, float margin);

private:// vars
    shared_ptr<SafetyEval> safety_eval_ptr_;
    float obs_avoidance_vel_ = 0.5f;
    bool log_to_file = false;

    // we make this recursive so that DroneControlBase subclass can grab StatusLock then call a 
    // base class method on DroneControlBase that also grabs the StatusLock.
    std::recursive_mutex status_mutex_;
};

}} //namespace
#endif

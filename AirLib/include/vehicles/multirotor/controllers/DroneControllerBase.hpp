// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DroneControlBase_hpp
#define air_DroneControlBase_hpp


#include "common/Common.hpp"
#include "controllers/VehicleControllerBase.hpp"
#include "common/common_utils/WorkerThread.hpp"
#include "common/Waiter.hpp"
#include "safety/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "DroneCommon.hpp"

namespace msr { namespace airlib {

/// DroneControllerBase represents a generic drone that can be controlled and queried for current state.
/// All control methods return a boolean where true means the command was completed successfully, and
/// false means the command was cancelled.  
/// 
/// Cancellable actions: all commands return a CancelableBase& cancelable_action object.
/// These objects can be used to cancel the command, or find out if the command has
/// been cancelled.  Only one command can be active at a time, so the next command you
/// send will automatically and immediately cancel any previous command that is in progress.
/// Many movement actions also take a duration, and when that duration is up the drone
/// will automatically hover at whatever location it is at.
///
/// NOTE: It is very important to understand the local position coordinate system.
/// All x,y,z values below are in North/East/Down NED coordinates.  This means the
/// z values will be more negative the higher you go.  Ground is zero, and then it is
/// negative from there up.  This is the defacto standard in the drone world so it is
/// simpler to keep with that throughout the AirSim library.
/// 
/// NOTE: all the movement commands, except hover, require you first call enableApiControl(true).  If that succeeds
/// it means your application now has control over the drone and the user cannot control the drone unless they
/// flip a switch on their controller taking control back from your program.  If the user takes control back
/// and you call one of these methods, an exception will be thrown telling your program that it no longer
/// has control.  If you call enableApiControl(false) the drone will hover waiting for user RC input.
    

class DroneControllerBase : public VehicleControllerBase {
public: //types
    class UnsafeMoveException : public VehicleMoveException {
    public:
        const SafetyEval::EvalResult result;

        UnsafeMoveException(const SafetyEval::EvalResult result_val, const string message = "")
            : VehicleMoveException(message), result(result_val)
        {}
    };

    struct StatusLock {
        StatusLock(DroneControllerBase* drone)
            : drone_(drone), lock_(drone->status_mutex_)
        {
            unused(drone_);
        }

    private:
        DroneControllerBase* drone_;
        std::lock_guard<std::recursive_mutex> lock_;
    };

    enum class LandedState : uint {
        Landed = 0,
        Flying = 1
    };

public: //interface for outside world
    /// The drone must be armed before it will fly.  Set arm to true to arm the drone.  
    /// On some drones arming may cause the motors to spin on low throttle, this is normal.
    /// Set arm to false to disarm the drone.  This will disable the motors, so don't do that
    /// unless the drone is on the ground!  Arming the drone also sets the "home position"
    /// This home position is local position x=0,y=0,z=0.  You can also query what GPS location
    /// that is via getHomeGeoPoint.  
    virtual bool armDisarm(bool arm, CancelableBase& cancelable_action) = 0;

    /// When armed you can tell the drone to takeoff.  This will fly to a preset altitude (like 2.5 meters)
    /// above the home position.  Once the drone is safely in the air you can use other commands to fly from there.
    /// If the drone is already flying takeoff will be ignored.  Pass non-zer max_wait_seconds if you want the
    /// method to also wait until the takeoff altitude is achieved.
    virtual bool takeoff(float max_wait_seconds, CancelableBase& cancelable_action);

    /// At any point this command will disable offboard control and land the drone at the current GPS location.
    /// How quickly the drone descends is up to the drone.  Some models will descend slowly if they have no 
    /// lidar telling them how far it is to the ground, while others that can see the ground will descend more
    /// quickly until they get near the ground.  None of that behavior is defined in this API because it is 
    /// depends on what kind of hardware the drone has onboard.  Pass non-zer max_wait_seconds if you want the
    /// method to also wait until the drone reports it has landed, the timeout here is a bit tricky, depends
    /// on how high you are and what the drone's configured descent velocity is.  If you don't want to wait
    /// pass zero.  You can also periodically check getLandedState to see if it has landed.
    virtual bool land(float max_wait_seconds, CancelableBase& cancelable_action);

    /// This command is a safety measure, at any point this command will cancel offboard control and send the
    /// drone back to the launch point (or home position).  Most drones are also configured to climb to a safe
    /// altitude before doing that so they don't run into a tree on the way home.
    virtual bool goHome(CancelableBase& cancelable_action);

    /// Move the drone by controlling the angles (or attitude) of the drone, if you set pitch, roll to zero
    /// and z to the current z value then it is equivalent to a hover command.  A little bit of pitch can
    /// make the drone move forwards, a little bit of roll can make it move sideways.  The yaw control can
    /// make the drone spin around on the spot.  The duration says how long you want to apply these settings
    /// before reverting to a hover command.  So you can say "fly forwards slowly for 1 second" using 
    /// moveByAngle(0.1, 0, z, yaw, 1, ...).  The cancelable_action can be used to canel all actions.  In fact,
    /// every time you call another move* method you will automatically cancel any previous action that is
    /// happening.
    virtual bool moveByAngle(float pitch, float roll, float z, float yaw, float duration
        , CancelableBase& cancelable_action);


    /// Move the drone by controlling the velocity vector of the drone. A little bit of vx can
    /// make the drone move forwards, a little bit of vy can make it move sideways.  A bit of vz can move
    /// the drone up or down vertically.  The yaw_mode can set a specific yaw target, or tell the drone
    /// to move as a specified yaw rate.  The yaw rate command is handy if you want to do a slow 360 
    /// and capture a nice smooth panorama.  The duration says how long you want to apply these settings
    /// before reverting to a hover command.  So you can say "fly forwards slowly for 1 second" using 
    /// moveByVelocity(0.1, 0, 0, 1, ...).  The cancelable_action can be used to canel all actions.  In fact,
    /// every time you call another move* method you will automatically cancel any previous action that is
    /// happening.
    virtual bool moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode,
        CancelableBase& cancelable_action);


    /// Move the drone by controlling the velocity x,y of the drone but with a fixed altitude z. A little bit of vx can
    /// make the drone move forwards, a little bit of vy can make it move sideways. 
    /// The yaw_mode can set a specific yaw target, or tell the drone
    /// to move as a specified yaw rate.  The yaw rate command is handy if you want to do a slow 360 
    /// and capture a nice smooth panorama.  The duration says how long you want to apply these settings
    /// before reverting to a hover command.  So you can say "fly forwards slowly for 1 second" using 
    /// moveByVelocityZ(0.1, 0, z, 1, ...).  The cancelable_action can be used to canel all actions.  In fact,
    /// every time you call another move* method you will automatically cancel any previous action that is
    /// happening.
    virtual bool moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode,
        CancelableBase& cancelable_action);

    /// Move the drone along the given path at the given speed and yaw.  The lookahead argument will smooth this path
    /// by looking ahead from current location by a given number of meters, then it will try and move the drone to 
    /// that lookahead position, thereby smoothing any corners in the path.  The lookahead can also ensure the drone 
    /// doesn't stop and start at each vertex along the path.
    virtual bool moveOnPath(const vector<Vector3r>& path, float velocity, DrivetrainType drivetrain, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead, CancelableBase& cancelable_action);


    /// Move the drone to the absolution x, y, z local positions at given speed and yaw.  Remember z is negative.  
    /// Positive z is under ground.  Instead of moving to the yaw before starting, the drone will move to the yaw position 
    /// smoothly as it goes, which means for short paths it may not reach that target heading until it has already
    /// reached the end point at which time the drone will continue rotating until it reaches the desired heading.
    /// The lookahead argument will smooth the path that moves the drone from it's current position to the target
    /// postion by looking ahead from current location by a given number of meters, it keeps doing this iteratively 
    /// until it reaches the target position.  This ensures a smoother flight.
    virtual bool moveToPosition(float x, float y, float z, float velocity, DrivetrainType drivetrain,
        const YawMode& yaw_mode, float lookahead, float adaptive_lookahead, CancelableBase& cancelable_action);

    /// moveToZ is a shortcut for moveToPosition at the current x, y location.
    virtual bool moveToZ(float z, float velocity, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead, CancelableBase& cancelable_action);

    virtual bool moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode, CancelableBase& cancelable_action);

    /// Rotate the drone to the specified fixed heading (yaw) while remaining stationery at the current x, y, and z.
    virtual bool rotateToYaw(float yaw, float margin, CancelableBase& cancelable_action);

    /// Rotate the drone to the specified yaw rate while remaining stationery at the current x, y, and z.
    /// bugbug: why isn't it just rotate(yaw_mode) ?
    virtual bool rotateByYawRate(float yaw_rate, float duration, CancelableBase& cancelable_action);

    /// Hover at the current x, y, and z.  If the drone is moving when this is called, it will try
    /// and move back to the location it was at when this command was received and hover there.  
    virtual bool hover(CancelableBase& cancelable_action);

    /// get state from estimator
    virtual Kinematics::State getKinematicsEstimated() = 0;
    
    /// get the current local position in NED coordinate (x=North/y=East,z=Down) so z is negative.
    virtual Vector3r getPosition() = 0;


    /// Get the current velocity of the drone
    virtual Vector3r getVelocity() = 0;

    /// Get the current orientation (or attitude) of the drone as a Quaternion.
    virtual Quaternionr getOrientation() = 0;

    /// Get debug pose, meaning of which is dependent on application usage. For example,
    /// this could be pose of real vehicle from log playback.
    virtual Pose getDebugPose();

    /// get the current X and Y position
    Vector2r getPositionXY();

    /// Get the Z position (z starts at zero on the ground, and becomes more and more negative as you go up)
    float getZ();

    /// Get current state of the drone, is it landed or in the air
    virtual LandedState getLandedState() = 0;

    /// Assigned remote control to use for this controller, 
    /// -1 = onboard RC, 0+ = joystick ID available on OS
    virtual int getRemoteControlID() { return -1; }

    /// Get the current RC inputs when RC transmitter is talking to to flight controller
    virtual RCData getRCData() = 0;

    virtual RCData estimateRCTrims(CancelableBase& cancelable_action, float trimduration = 1, float minCountForTrim = 10, float maxTrim = 100);

    /// Set the RC data that should be used by flight controller
    virtual void setRCData(const RCData& rcData) = 0;

    /// Get the home point (where drone was armed before takeoff).  This is the location the drone 
    /// will return to if you call goHome().
    virtual GeoPoint getHomeGeoPoint() = 0;

    /// Get the current GPS location of the drone.
    virtual GeoPoint getGpsLocation() = 0;

    //below are for passing information from simulator to API layer
    //in non simulation mode default would be no collision unless
    //controller implements otherwise.
    virtual CollisionInfo getCollisionInfo();
    virtual void setCollisionInfo(const CollisionInfo& collision_info);

    //safety settings
    virtual void setSafetyEval(const shared_ptr<SafetyEval> safety_eval_ptr);
    virtual bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z);
    virtual const VehicleParams& getVehicleParams() = 0;

    //*********************************common pre & post for move commands***************************************************
    //TODO: make these protected
    virtual bool loopCommandPre();
    virtual void loopCommandPost();
    //*********************************common pre & post for move commands***************************************************

    DroneControllerBase() = default;
    virtual ~DroneControllerBase() = default;

protected: //must implement interface by derived class
    //low level commands
    //all angles in degrees, lengths in meters, velocities in m/s, durations in seconds
    //all coordinates systems are world NED (+x is North, +y is East, +z is down)
    virtual void commandRollPitchZ(float pitch, float roll, float z, float yaw) = 0;
    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) = 0;
    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) = 0;
    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) = 0;

    //config commands
    virtual float getCommandPeriod() = 0; //time between two command required for drone in seconds
    virtual float getTakeoffZ() = 0;  // the height above ground for the drone after successful takeoff (Z above ground is negative due to NED coordinate system).
                                      //noise in difference of two position coordinates. This is not GPS or position accuracy which can be very low such as 1m.
                                      //the difference between two position cancels out transitional errors. Typically this would be 0.1m or lower.
    virtual float getDistanceAccuracy() = 0; 

protected: //optional oveerides recommanded for any drones, default implementation may work
    virtual float getAutoLookahead(float velocity, float adaptive_lookahead,
        float max_factor = 40, float min_factor = 30);
    virtual float getObsAvoidanceVelocity(float risk_dist, float max_obs_avoidance_vel);

protected: //utility functions and data members for derived classes
    typedef std::function<bool()> WaitFunction;

    // helper function can wait for anything (as defined by the given function) up to the max_wait duration (in seconds).
    // returns true if the wait function succeeded, or false if timeout occurred or the timeout is invalid.
    virtual bool waitForFunction(WaitFunction function, float max_wait, CancelableBase& cancelable_action);

    //useful for derived class to check after takeoff
    virtual bool waitForZ(float max_wait_seconds, float z, float margin, CancelableBase& cancelable_action);


    //*********************************safe wrapper around low level commands***************************************************
    virtual bool moveByVelocity(float vx, float vy, float vz, const YawMode& yaw_mode);
    virtual bool moveByVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode);
    virtual bool moveToPosition(const Vector3r& dest, const YawMode& yaw_mode);
    virtual bool moveByRollPitchZ(float pitch, float roll, float z, float yaw);
    //****************************************************************************************************************************

    /************* safety checks & emergency manuevers ************/
    virtual bool emergencyManeuverIfUnsafe(const SafetyEval::EvalResult& result);
    virtual bool safetyCheckVelocity(const Vector3r& velocity);
    virtual bool safetyCheckVelocityZ(float vx, float vy, float z);
    virtual bool safetyCheckDestination(const Vector3r& dest_loc);
    /************* safety checks & emergency manuevers ************/

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

    RCData rc_data_trims_;

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

    CollisionInfo collision_info_;

    // we make this recursive so that DroneControllerBase subclass can grab StatusLock then call a 
    // base class method on DroneControllerBase that also grabs the StatusLock.
    std::recursive_mutex status_mutex_;
};

}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_MavLinkDroneController_hpp
#define msr_airlib_MavLinkDroneController_hpp

#include "MavLinkConnection.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkNode.hpp"
#include "MavLinkVehicle.hpp"
#include "MavLinkVideoStream.hpp"

#include <exception>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "common/AirSimSettings.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/PidController.hpp"
#include "common/VectorMath.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/SmoothingFilter.hpp"
#include "common/common_utils/Timer.hpp"
#include "physics/World.hpp"
#include "sensors/SensorCollection.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"

//sensors
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/distance/DistanceSimple.hpp"
#include "sensors/gps/GpsBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"

namespace msr
{
namespace airlib
{

    class MavLinkMultirotorApi : public MultirotorApiBase
    {
    public: //methods
        virtual ~MavLinkMultirotorApi()
        {
            closeAllConnection();
            if (this->connect_thread_.joinable()) {
                this->connect_thread_.join();
            }
            if (this->telemetry_thread_.joinable()) {
                this->telemetry_thread_.join();
            }
        }

        //non-base interface specific to MavLinKDroneController
        void initialize(const AirSimSettings::MavLinkConnectionInfo& connection_info, const SensorCollection* sensors, bool is_simulation)
        {
            connection_info_ = connection_info;
            sensors_ = sensors;
            is_simulation_mode_ = is_simulation;
            lock_step_enabled_ = connection_info.lock_step;
            try {
                openAllConnections();
                is_ready_ = true;
            }
            catch (std::exception& ex) {
                is_ready_ = false;
                is_ready_message_ = Utils::stringf("Failed to connect: %s", ex.what());
            }
        }

        Pose getMocapPose()
        {
            std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
            return mocap_pose_;
        }

        virtual const SensorCollection& getSensors() const override
        {
            return *sensors_;
        }

        //reset PX4 stack
        virtual void resetImplementation() override
        {
            // note this is called AFTER "initialize" when we've connected to the drone
            // so this method cannot reset any of that connection state.
            world_ = nullptr;

            for (UpdatableObject* container = this->getParent(); container != nullptr; container = container->getParent()) {
                if (container->getName() == "World") {
                    // cool beans!
                    // note: cannot use dynamic_cast because Unreal builds with /GR- for some unknown reason...
                    world_ = static_cast<World*>(container);
                }
            }
            MultirotorApiBase::resetImplementation();
            was_reset_ = true;
        }

        unsigned long long getSimTime()
        {
            // This ensures HIL_SENSOR and HIL_GPS have matching clocks.
            if (lock_step_active_) {
                if (sim_time_us_ == 0) {
                    sim_time_us_ = clock()->nowNanos() / 1000;
                }
                return sim_time_us_;
            }
            else {
                return clock()->nowNanos() / 1000;
            }
        }

        void advanceTime()
        {
            sim_time_us_ = clock()->nowNanos() / 1000;
        }

        //update sensors in PX4 stack
        virtual void update() override
        {
            try {
                auto now = clock()->nowNanos() / 1000;
                MultirotorApiBase::update();

                if (sensors_ == nullptr || !connected_ || connection_ == nullptr || !connection_->isOpen() || !got_first_heartbeat_)
                    return;

                {
                    std::lock_guard<std::mutex> guard(telemetry_mutex_);
                    update_count_++;
                }

                if (lock_step_active_) {
                    if (last_update_time_ + 1000000 < now) {
                        // if 1 second passes then something is terribly wrong, reset lockstep mode
                        lock_step_active_ = false;
                        {
                            std::lock_guard<std::mutex> guard(telemetry_mutex_);
                            lock_step_resets_++;
                        }
                        addStatusMessage("timeout on HilActuatorControlsMessage, resetting lock step mode");
                    }
                    else if (!received_actuator_controls_) {
                        // drop this one since we are in LOCKSTEP mode and we have not yet received the HilActuatorControlsMessage.
                        return;
                    }
                }

                last_update_time_ = now;

                {
                    std::lock_guard<std::mutex> guard(telemetry_mutex_);
                    hil_sensor_count_++;
                }
                advanceTime();

                //send sensor updates
                const auto& imu_output = getImuData("");
                const auto& mag_output = getMagnetometerData("");
                const auto& baro_output = getBarometerData("");

                sendHILSensor(imu_output.linear_acceleration,
                              imu_output.angular_velocity,
                              mag_output.magnetic_field_body,
                              baro_output.pressure * 0.01f /*Pa to Millibar */,
                              baro_output.altitude);

                sendSystemTime();

                const uint count_distance_sensors = getSensors().size(SensorBase::SensorType::Distance);
                if (count_distance_sensors != 0) {
                    const auto* distance_sensor = static_cast<const DistanceSimple*>(
                        sensors_->getByType(SensorBase::SensorType::Distance));
                    // Don't send the data if sending to external controller is disabled in settings
                    if (distance_sensor && distance_sensor->getParams().external_controller) {
                        const auto& distance_output = distance_sensor->getOutput();

                        sendDistanceSensor(distance_output.min_distance * 100, //m -> cm
                                           distance_output.max_distance * 100, //m -> cm
                                           distance_output.distance * 100, //m-> cm
                                           0, //sensor type: //TODO: allow changing in settings?
                                           77, //sensor id, //TODO: should this be something real?
                                           distance_output.relative_pose.orientation); //TODO: convert from radians to degrees?
                    }
                }

                const uint count_gps_sensors = getSensors().size(SensorBase::SensorType::Gps);
                if (count_gps_sensors != 0) {
                    const auto& gps_output = getGpsData("");

                    //send GPS
                    if (gps_output.is_valid && gps_output.gnss.time_utc > last_gps_time_) {
                        last_gps_time_ = gps_output.gnss.time_utc;
                        Vector3r gps_velocity = gps_output.gnss.velocity;
                        Vector3r gps_velocity_xy = gps_velocity;
                        gps_velocity_xy.z() = 0;
                        float gps_cog;
                        if (Utils::isApproximatelyZero(gps_velocity.y(), 1E-2f) && Utils::isApproximatelyZero(gps_velocity.x(), 1E-2f))
                            gps_cog = 0;
                        else
                            gps_cog = Utils::radiansToDegrees(atan2(gps_velocity.y(), gps_velocity.x()));
                        if (gps_cog < 0)
                            gps_cog += 360;

                        sendHILGps(gps_output.gnss.geo_point, gps_velocity, gps_velocity_xy.norm(), gps_cog, gps_output.gnss.eph, gps_output.gnss.epv, gps_output.gnss.fix_type, 10);
                    }
                }

                auto end = clock()->nowNanos() / 1000;
                {
                    std::lock_guard<std::mutex> guard(telemetry_mutex_);
                    update_time_ += (end - now);
                }
            }
            catch (std::exception& e) {
                addStatusMessage("Exception sending messages to vehicle");
                addStatusMessage(e.what());
                disconnect();
                connect(); // re-start a new connection so PX4 can be restarted and AirSim will happily continue on.
            }

            //must be done at the end
            if (was_reset_)
                was_reset_ = false;
        }

        virtual bool isReady(std::string& message) const override
        {
            if (!is_ready_ && is_ready_message_.size() > 0) {
                message = is_ready_message_;
            }
            return is_ready_;
        }

        virtual bool canArm() const override
        {
            return is_ready_ && has_gps_lock_;
        }

        //TODO: this method can't be const yet because it clears previous messages
        virtual void getStatusMessages(std::vector<std::string>& messages) override
        {
            updateState();

            //clear param
            messages.clear();

            //move messages from private vars to param
            std::lock_guard<std::mutex> guard(status_text_mutex_);
            while (!status_messages_.empty()) {
                messages.push_back(status_messages_.front());
                status_messages_.pop();
            }
        }

        virtual Kinematics::State getKinematicsEstimated() const override
        {
            updateState();
            Kinematics::State state;
            //TODO: reduce code duplication in getPosition() etc methods?
            state.pose.position = Vector3r(current_state_.local_est.pos.x, current_state_.local_est.pos.y, current_state_.local_est.pos.z);
            state.pose.orientation = VectorMath::toQuaternion(current_state_.attitude.pitch, current_state_.attitude.roll, current_state_.attitude.yaw);
            state.twist.linear = Vector3r(current_state_.local_est.lin_vel.x, current_state_.local_est.lin_vel.y, current_state_.local_est.lin_vel.z);
            state.twist.angular = Vector3r(current_state_.attitude.roll_rate, current_state_.attitude.pitch_rate, current_state_.attitude.yaw_rate);
            state.accelerations.linear = Vector3r(current_state_.local_est.acc.x, current_state_.local_est.acc.y, current_state_.local_est.acc.z);
            //TODO: how do we get angular acceleration?
            return state;
        }

        virtual bool isApiControlEnabled() const override
        {
            return is_api_control_enabled_;
        }

        virtual void enableApiControl(bool is_enabled) override
        {
            checkValidVehicle();
            if (is_enabled) {
                mav_vehicle_->requestControl();
                is_api_control_enabled_ = true;
            }
            else {
                mav_vehicle_->releaseControl();
                is_api_control_enabled_ = false;
            }
        }

        virtual Vector3r getPosition() const override
        {
            updateState();
            return Vector3r(current_state_.local_est.pos.x, current_state_.local_est.pos.y, current_state_.local_est.pos.z);
        }

        virtual Vector3r getVelocity() const override
        {
            updateState();
            return Vector3r(current_state_.local_est.lin_vel.x, current_state_.local_est.lin_vel.y, current_state_.local_est.lin_vel.z);
        }

        virtual Quaternionr getOrientation() const override
        {
            updateState();
            return VectorMath::toQuaternion(current_state_.attitude.pitch, current_state_.attitude.roll, current_state_.attitude.yaw);
        }

        virtual LandedState getLandedState() const override
        {
            updateState();
            return current_state_.controls.landed ? LandedState::Landed : LandedState::Flying;
        }

        virtual real_T getActuation(unsigned int rotor_index) const override
        {
            if (!is_simulation_mode_)
                throw std::logic_error("Attempt to read motor controls while not in simulation mode");

            std::lock_guard<std::mutex> guard(hil_controls_mutex_);
            return rotor_controls_[rotor_index];
        }

        virtual size_t getActuatorCount() const override
        {
            return RotorControlsCount;
        }

        virtual bool armDisarm(bool arm) override
        {
            SingleCall lock(this);

            checkValidVehicle();
            bool rc = false;
            if (arm) {
                float timeout_sec = 10;
                waitForHomeLocation(timeout_sec);
                waitForStableGroundPosition(timeout_sec);
            }

            mav_vehicle_->armDisarm(arm).wait(10000, &rc);
            return rc;
        }

        void onArmed()
        {
            if (connection_info_.logs.size() > 0 && mav_vehicle_ != nullptr) {
                auto con = mav_vehicle_->getConnection();
                if (con != nullptr) {
                    if (log_ != nullptr) {
                        log_->close();
                        log_ = nullptr;
                    }

                    try {
                        std::time_t t = std::time(0); // get time now
                        std::tm* now = std::localtime(&t);
                        auto folder = Utils::stringf("%04d-%02d-%02d", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday);
                        auto path = common_utils::FileSystem::ensureFolder(connection_info_.logs, folder);
                        auto filename = Utils::stringf("%02d-%02d-%02d.mavlink", now->tm_hour, now->tm_min, now->tm_sec);
                        auto fullpath = common_utils::FileSystem::combine(path, filename);
                        addStatusMessage(Utils::stringf("Opening log file: %s", fullpath.c_str()));
                        log_file_name_ = fullpath;
                        log_ = std::make_shared<mavlinkcom::MavLinkFileLog>();
                        log_->openForWriting(fullpath, false);
                        con->startLoggingSendMessage(log_);
                        con->startLoggingReceiveMessage(log_);
                        if (con != connection_) {
                            // log the SITL channel also
                            connection_->startLoggingSendMessage(log_);
                            connection_->startLoggingReceiveMessage(log_);
                        }
                        start_telemtry_thread();
                    }
                    catch (std::exception& ex) {
                        addStatusMessage(std::string("Opening log file failed: ") + ex.what());
                    }
                }
            }
        }

        void onDisarmed()
        {
            if (connection_info_.logs.size() > 0 && mav_vehicle_ != nullptr) {
                auto con = mav_vehicle_->getConnection();
                if (con != nullptr) {
                    con->stopLoggingSendMessage();
                    con->stopLoggingReceiveMessage();
                }
                if (connection_ != nullptr) {
                    connection_->stopLoggingSendMessage();
                    connection_->stopLoggingReceiveMessage();
                }
            }
            if (log_ != nullptr) {
                addStatusMessage(Utils::stringf("Closing log file: %s", log_file_name_.c_str()));
                log_->close();
                log_ = nullptr;
            }
        }

        void waitForHomeLocation(float timeout_sec)
        {
            if (!current_state_.home.is_set) {
                addStatusMessage("Waiting for valid GPS home location...");
                if (!waitForFunction([&]() {
                         return current_state_.home.is_set;
                     },
                                     timeout_sec)
                         .isComplete()) {
                    throw VehicleMoveException("Vehicle does not have a valid GPS home location");
                }
            }
        }

        void waitForStableGroundPosition(float timeout_sec)
        {
            // wait for ground stabilization
            if (ground_variance_ > GroundTolerance) {
                addStatusMessage("Waiting for z-position to stabilize...");
                if (!waitForFunction([&]() {
                         return ground_variance_ <= GroundTolerance;
                     },
                                     timeout_sec)
                         .isComplete()) {
                    auto msg = Utils::stringf("Ground is not stable, variance is %f", ground_variance_);
                    throw VehicleMoveException(msg);
                }
            }
        }

        virtual bool takeoff(float timeout_sec) override
        {
            SingleCall lock(this);

            checkValidVehicle();

            waitForHomeLocation(timeout_sec);
            waitForStableGroundPosition(timeout_sec);

            bool rc = false;
            auto vec = getPosition();
            auto yaw = current_state_.attitude.yaw;
            float z = vec.z() + getTakeoffZ();
            if (!mav_vehicle_->takeoff(z, 0.0f /* pitch */, yaw).wait(static_cast<int>(timeout_sec * 1000), &rc)) {
                throw VehicleMoveException("TakeOff command - timeout waiting for response");
            }
            if (!rc) {
                throw VehicleMoveException("TakeOff command rejected by drone");
            }
            if (timeout_sec <= 0)
                return true; // client doesn't want to wait.

            return waitForZ(timeout_sec, z, getDistanceAccuracy());
        }

        virtual bool land(float timeout_sec) override
        {
            SingleCall lock(this);

            //TODO: bugbug: really need a downward pointing distance to ground sensor to do this properly, for now
            //we assume the ground is relatively flat an we are landing roughly at the home altitude.
            updateState();
            checkValidVehicle();
            if (current_state_.home.is_set) {
                bool rc = false;
                if (!mav_vehicle_->land(current_state_.global_est.pos.lat, current_state_.global_est.pos.lon, current_state_.home.global_pos.alt).wait(10000, &rc)) {
                    throw VehicleMoveException("Landing command - timeout waiting for response from drone");
                }
                else if (!rc) {
                    throw VehicleMoveException("Landing command rejected by drone");
                }
            }
            else {
                throw VehicleMoveException("Cannot land safely with out a home position that tells us the home altitude.  Could fix this if we hook up a distance to ground sensor...");
            }

            const auto& waiter = waitForFunction([&]() {
                updateState();
                return current_state_.controls.landed;
            },
                                                 timeout_sec);

            // Wait for landed state (or user cancellation)
            if (!waiter.isComplete()) {
                throw VehicleMoveException("Drone hasn't reported a landing state");
            }
            return waiter.isComplete();
        }

        virtual bool goHome(float timeout_sec) override
        {
            SingleCall lock(this);

            checkValidVehicle();
            bool rc = false;
            if (mav_vehicle_ != nullptr && !mav_vehicle_->returnToHome().wait(
                                               static_cast<int>(timeout_sec) * 1000, &rc)) {
                throw VehicleMoveException("goHome - timeout waiting for response from drone");
            }
            return rc;
        }

        virtual bool moveToPosition(float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain,
                                    const YawMode& yaw_mode, float lookahead, float adaptive_lookahead) override
        {
            SingleTaskCall lock(this);

            unused(adaptive_lookahead);
            unused(lookahead);
            unused(drivetrain);

            // save current manual, cruise, and max velocity parameters
            bool result = false;
            mavlinkcom::MavLinkParameter manual_velocity_parameter, cruise_velocity_parameter, max_velocity_parameter;
            result = mav_vehicle_->getParameter("MPC_VEL_MANUAL").wait(1000, &manual_velocity_parameter);
            result = result && mav_vehicle_->getParameter("MPC_XY_CRUISE").wait(1000, &cruise_velocity_parameter);
            result = result && mav_vehicle_->getParameter("MPC_XY_VEL_MAX").wait(1000, &max_velocity_parameter);

            if (result) {
                // set max velocity parameter
                mavlinkcom::MavLinkParameter p;
                p.name = "MPC_XY_VEL_MAX";
                p.value = velocity;
                mav_vehicle_->setParameter(p).wait(1000, &result);

                if (result) {
                    const Vector3r& goal_pos = Vector3r(x, y, z);
                    Vector3r goal_dist_vect;
                    float goal_dist;

                    Waiter waiter(getCommandPeriod(), timeout_sec, getCancelToken());

                    while (!waiter.isComplete()) {
                        goal_dist_vect = getPosition() - goal_pos;
                        const Vector3r& goal_normalized = goal_dist_vect.normalized();
                        goal_dist = goal_dist_vect.dot(goal_normalized);

                        if (goal_dist > getDistanceAccuracy()) {
                            moveToPositionInternal(goal_pos, yaw_mode);

                            //sleep for rest of the cycle
                            if (!waiter.sleep())
                                return false;
                        }
                        else {
                            waiter.complete();
                        }
                    }

                    // reset manual, cruise, and max velocity parameters
                    bool result_temp = false;
                    mav_vehicle_->setParameter(manual_velocity_parameter).wait(1000, &result);
                    mav_vehicle_->setParameter(cruise_velocity_parameter).wait(1000, &result_temp);
                    result = result && result_temp;
                    mav_vehicle_->setParameter(max_velocity_parameter).wait(1000, &result_temp);
                    result = result && result_temp;

                    return result && waiter.isComplete();
                }
            }

            return result;
        }

        virtual bool hover() override
        {
            SingleCall lock(this);

            bool rc = false;
            checkValidVehicle();
            mavlinkcom::AsyncResult<bool> result = mav_vehicle_->loiter();
            //auto start_time = std::chrono::system_clock::now();
            while (!getCancelToken().isCancelled()) {
                if (result.wait(100, &rc)) {
                    break;
                }
            }
            return rc;
        }

        virtual GeoPoint getHomeGeoPoint() const override
        {
            updateState();
            if (current_state_.home.is_set)
                return GeoPoint(current_state_.home.global_pos.lat, current_state_.home.global_pos.lon, current_state_.home.global_pos.alt);
            else
                return GeoPoint(Utils::nan<double>(), Utils::nan<double>(), Utils::nan<float>());
        }

        virtual GeoPoint getGpsLocation() const override
        {
            updateState();
            return GeoPoint(current_state_.global_est.pos.lat, current_state_.global_est.pos.lon, current_state_.global_est.pos.alt);
        }

        virtual void sendTelemetry(float last_interval = -1) override
        {
            if (connection_ == nullptr || mav_vehicle_ == nullptr) {
                return;
            }

            // This method is called at high frequence from MultirotorPawnSimApi::updateRendering.
            mavlinkcom::MavLinkTelemetry data;
            connection_->getTelemetry(data);
            if (data.messages_received == 0) {
                if (!hil_message_timer_.started()) {
                    hil_message_timer_.start();
                }
                else if (hil_message_timer_.seconds() > messageReceivedTimeout) {
                    addStatusMessage("not receiving any messages from HIL, please restart your HIL node and try again");
                    hil_message_timer_.stop();
                }
            }
            else {
                hil_message_timer_.stop();
            }
        }

        void writeTelemetry(float last_interval = -1)
        {
            auto proxy = logviewer_proxy_;
            auto log = log_;

            if ((logviewer_proxy_ == nullptr && log_ == nullptr)) {
                return;
            }

            mavlinkcom::MavLinkTelemetry data;
            connection_->getTelemetry(data);

            // listen to the other mavlink connection also
            auto mavcon = mav_vehicle_->getConnection();
            if (mavcon != connection_) {
                mavlinkcom::MavLinkTelemetry gcs;
                mavcon->getTelemetry(gcs);

                data.handler_microseconds += gcs.handler_microseconds;
                data.messages_handled += gcs.messages_handled;
                data.messages_received += gcs.messages_received;
                data.messages_sent += gcs.messages_sent;

                if (gcs.messages_received == 0) {
                    if (!gcs_message_timer_.started()) {
                        gcs_message_timer_.start();
                    }
                    else if (gcs_message_timer_.seconds() > messageReceivedTimeout) {
                        addStatusMessage("not receiving any messages from GCS port, please restart your SITL node and try again");
                    }
                }
                else {
                    gcs_message_timer_.stop();
                }
            }

            data.render_time = static_cast<int64_t>(last_interval * 1000000); // microseconds

            {
                std::lock_guard<std::mutex> guard(telemetry_mutex_);
                uint32_t average_delay = 0;
                uint32_t average_update = 0;
                if (hil_sensor_count_ != 0) {
                    average_delay = actuator_delay_ / hil_sensor_count_;
                    average_update = static_cast<uint32_t>(update_time_ / hil_sensor_count_);
                }

                data.update_rate = update_count_;
                data.sensor_rate = hil_sensor_count_;
                data.actuation_delay = average_delay;
                data.lock_step_resets = lock_step_resets_;
                data.update_time = average_update;
                // reset the counters we just captured.
                update_count_ = 0;
                hil_sensor_count_ = 0;
                actuator_delay_ = 0;
                update_time_ = 0;
            }

            if (proxy != nullptr) {
                proxy->sendMessage(data);
            }

            if (log != nullptr) {
                mavlinkcom::MavLinkMessage msg;
                msg.magic = MAVLINK_STX_MAVLINK1;
                data.encode(msg);
                msg.update_checksum();
                // disk I/O is unpredictable, so we have to get it out of the update loop
                // which is why this thread exists.
                log->write(msg);
            }
        }

        void start_telemtry_thread()
        {
            if (this->telemetry_thread_.joinable()) {
                this->telemetry_thread_.join();
            }

            // reset the counters we use for telemetry.
            update_count_ = 0;
            hil_sensor_count_ = 0;
            actuator_delay_ = 0;

            this->telemetry_thread_ = std::thread(&MavLinkMultirotorApi::telemtry_thread, this);
        }

        void telemtry_thread()
        {
            while (log_ != nullptr) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                writeTelemetry(1);
            }
        }

        virtual float getCommandPeriod() const override
        {
            return 1.0f / 50; //1 period of 50hz
        }

        virtual float getTakeoffZ() const override
        {
            // pick a number, PX4 doesn't have a fixed limit here, but 3 meters is probably safe
            // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
            return -3.0f;
        }

        virtual float getDistanceAccuracy() const override
        {
            return 0.5f; //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
        }

    protected: //methods
        virtual void setControllerGains(uint8_t controllerType, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
        {
            unused(controllerType);
            unused(kp);
            unused(ki);
            unused(kd);
            Utils::log("Not Implemented: setControllerGains", Utils::kLogLevelInfo);
        }

        virtual void commandMotorPWMs(float front_right_pwm, float front_left_pwm, float rear_right_pwm, float rear_left_pwm) override
        {
            unused(front_right_pwm);
            unused(front_left_pwm);
            unused(rear_right_pwm);
            unused(rear_left_pwm);
            Utils::log("Not Implemented: commandMotorPWMs", Utils::kLogLevelInfo);
        }

        virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) override
        {
            if (target_height_ != -z) {
                // these PID values were calculated experimentally using AltHoldCommand n MavLinkTest, this provides the best
                // control over thrust to achieve minimal over/under shoot in a reasonable amount of time, but it has not
                // been tested on a real drone outside jMavSim, so it may need recalibrating...
                thrust_controller_.setPoint(-z, .05f, .005f, 0.09f);
                target_height_ = -z;
            }
            checkValidVehicle();
            auto state = mav_vehicle_->getVehicleState();
            float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
            mav_vehicle_->moveByAttitude(roll, pitch, yaw, 0, 0, 0, thrust);
        }
        virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) override
        {
            if (target_height_ != -z) {
                thrust_controller_.setPoint(-z, .05f, .005f, 0.09f);
                target_height_ = -z;
            }
            checkValidVehicle();
            auto state = mav_vehicle_->getVehicleState();
            float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
            mav_vehicle_->moveByAttitude(roll, pitch, 0, 0, 0, yaw_rate, thrust);
        }

        virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) override
        {
            checkValidVehicle();
            mav_vehicle_->moveByAttitude(roll, pitch, yaw, 0, 0, 0, throttle);
        }

        virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) override
        {
            checkValidVehicle();
            mav_vehicle_->moveByAttitude(roll, pitch, 0, 0, 0, yaw_rate, throttle);
        }

        virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) override
        {
            if (target_height_ != -z) {
                thrust_controller_.setPoint(-z, .05f, .005f, 0.09f);
                target_height_ = -z;
            }
            checkValidVehicle();
            auto state = mav_vehicle_->getVehicleState();
            float thrust = 0.21f + thrust_controller_.control(-state.local_est.pos.z);
            mav_vehicle_->moveByAttitude(0, 0, 0, roll_rate, pitch_rate, yaw_rate, thrust);
        }

        virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) override
        {
            checkValidVehicle();
            mav_vehicle_->moveByAttitude(0, 0, 0, roll_rate, pitch_rate, yaw_rate, throttle);
        }

        virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
        {
            checkValidVehicle();
            float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
            mav_vehicle_->moveByLocalVelocity(vx, vy, vz, !yaw_mode.is_rate, yaw);
        }

        virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
        {
            checkValidVehicle();
            float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
            mav_vehicle_->moveByLocalVelocityWithAltHold(vx, vy, z, !yaw_mode.is_rate, yaw);
        }

        virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
        {
            checkValidVehicle();
            float yaw = yaw_mode.yaw_or_rate * M_PIf / 180;
            mav_vehicle_->moveToLocalPosition(x, y, z, !yaw_mode.is_rate, yaw);
        }

        //TODO: decouple MultirotorApiBase, VehicalParams and SafetyEval
        virtual const MultirotorApiParams& getMultirotorApiParams() const override
        {
            //defaults are good for PX4 generic quadcopter.
            static const MultirotorApiParams vehicle_params_;
            return vehicle_params_;
        }

        virtual void beforeTask() override
        {
            startOffboardMode();
        }

        virtual void afterTask() override
        {
            endOffboardMode();
        }

    public:
        class MavLinkLogViewerLog : public mavlinkcom::MavLinkLog
        {
        public:
            MavLinkLogViewerLog(std::shared_ptr<mavlinkcom::MavLinkNode> proxy)
            {
                proxy_ = proxy;
            }

            ~MavLinkLogViewerLog()
            {
                proxy_ = nullptr;
            }

            void write(const mavlinkcom::MavLinkMessage& msg, uint64_t timestamp = 0) override
            {
                if (proxy_ != nullptr) {
                    unused(timestamp);
                    mavlinkcom::MavLinkMessage copy;
                    ::memcpy(&copy, &msg, sizeof(mavlinkcom::MavLinkMessage));
                    try {
                        proxy_->sendMessage(copy);
                    }
                    catch (std::exception&) {
                        failures++;
                        if (failures == 100) {
                            // hmmm, doesn't like the proxy, bad socket perhaps, so stop trying...
                            proxy_ = nullptr;
                        }
                    }
                }
            }

        private:
            std::shared_ptr<mavlinkcom::MavLinkNode> proxy_;
            int failures = 0;
        };

    protected: //methods
        virtual void connect()
        {
            if (!connecting_) {
                connecting_ = true;
                if (this->connect_thread_.joinable()) {
                    this->connect_thread_.join();
                }
                this->connect_thread_ = std::thread(&MavLinkMultirotorApi::connect_thread, this);
            }
        }

        virtual void disconnect()
        {
            addStatusMessage("Disconnecting mavlink vehicle");
            connected_ = false;
            connecting_ = false;

            if (is_armed_) {
                // close the telemetry log.
                is_armed_ = false;
                onDisarmed();
            }

            if (connection_ != nullptr) {
                if (is_hil_mode_set_ && mav_vehicle_ != nullptr) {
                    setNormalMode();
                }

                connection_->stopLoggingSendMessage();
                connection_->stopLoggingReceiveMessage();
                connection_->close();
            }

            if (hil_node_ != nullptr) {
                hil_node_->close();
            }

            if (mav_vehicle_ != nullptr) {
                auto c = mav_vehicle_->getConnection();
                if (c != nullptr) {
                    c->stopLoggingSendMessage();
                    c->stopLoggingReceiveMessage();
                }
                mav_vehicle_->close();
                mav_vehicle_ = nullptr;
            }

            if (video_server_ != nullptr)
                video_server_->close();

            if (logviewer_proxy_ != nullptr) {
                logviewer_proxy_->close();
                logviewer_proxy_ = nullptr;
            }

            if (logviewer_out_proxy_ != nullptr) {
                logviewer_out_proxy_->close();
                logviewer_out_proxy_ = nullptr;
            }

            if (qgc_proxy_ != nullptr) {
                qgc_proxy_->close();
                qgc_proxy_ = nullptr;
            }

            resetState();
        }

        void connect_thread()
        {
            addStatusMessage("Waiting for mavlink vehicle...");
            connecting_ = true;
            createMavConnection(connection_info_);
            if (mav_vehicle_ != nullptr) {
                connectToLogViewer();
                connectToQGC();
            }

            if (connecting_) {
                // only set connected if we haven't already been told to disconnect.
                connecting_ = false;
                connected_ = true;
            }
        }

        virtual void close()
        {
            disconnect();
        }

        void closeAllConnection()
        {
            close();
        }

    private: //methods
        void openAllConnections()
        {
            Utils::log("Opening mavlink connection");
            close(); //just in case if connections were open
            resetState(); //reset all variables we might have changed during last session
            connect();
        }

        void getMocapPose(Vector3r& position, Quaternionr& orientation) const
        {
            position.x() = MocapPoseMessage.x;
            position.y() = MocapPoseMessage.y;
            position.z() = MocapPoseMessage.z;
            orientation.w() = MocapPoseMessage.q[0];
            orientation.x() = MocapPoseMessage.q[1];
            orientation.y() = MocapPoseMessage.q[2];
            orientation.z() = MocapPoseMessage.q[3];
        }

        //TODO: this method used to send collision to external sim. Do we still need this?
        void sendCollision(float normalX, float normalY, float normalZ)
        {
            checkValidVehicle();

            mavlinkcom::MavLinkCollision collision{};
            collision.src = 1; //provider of data is MavLink system in id field
            collision.id = mav_vehicle_->getLocalSystemId();
            collision.action = static_cast<uint8_t>(mavlinkcom::MAV_COLLISION_ACTION::MAV_COLLISION_ACTION_REPORT);
            collision.threat_level = static_cast<uint8_t>(mavlinkcom::MAV_COLLISION_THREAT_LEVEL::MAV_COLLISION_THREAT_LEVEL_NONE);
            // we are abusing these fields, passing the angle of the object we hit, so that jMAVSim knows how to bounce off.
            collision.time_to_minimum_delta = normalX;
            collision.altitude_minimum_delta = normalY;
            collision.horizontal_minimum_delta = normalZ;
            mav_vehicle_->sendMessage(collision);
        }

        //TODO: do we still need this method?
        bool hasVideoRequest()
        {
            mavlinkcom::MavLinkVideoServer::MavLinkVideoRequest image_req;
            return video_server_->hasVideoRequest(image_req);
        }

        //TODO: do we still need this method?
        void sendImage(unsigned char data[], uint32_t length, uint16_t width, uint16_t height)
        {
            const int MAVLINK_DATA_STREAM_IMG_PNG = 6;
            video_server_->sendFrame(data, length, width, height, MAVLINK_DATA_STREAM_IMG_PNG, 0);
        }

        //put PX4 in normal mode (i.e. non-simulation mode)
        void setNormalMode()
        {
            if (is_hil_mode_set_ && connection_ != nullptr && mav_vehicle_ != nullptr) {

                // remove MAV_MODE_FLAG_HIL_ENABLED flag from current mode
                std::lock_guard<std::mutex> guard(set_mode_mutex_);
                int mode = mav_vehicle_->getVehicleState().mode;
                mode &= ~static_cast<int>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_HIL_ENABLED);

                mavlinkcom::MavCmdDoSetMode cmd;
                cmd.command = static_cast<uint16_t>(mavlinkcom::MAV_CMD::MAV_CMD_DO_SET_MODE);
                cmd.Mode = static_cast<float>(mode);
                mav_vehicle_->sendCommand(cmd);

                is_hil_mode_set_ = false;
            }
        }

        //put PX4 in simulation mode
        void setHILMode()
        {
            if (!is_simulation_mode_)
                throw std::logic_error("Attempt to set device in HIL mode while not in simulation mode");

            checkValidVehicle();

            // add MAV_MODE_FLAG_HIL_ENABLED flag to current mode
            std::lock_guard<std::mutex> guard(set_mode_mutex_);
            int mode = mav_vehicle_->getVehicleState().mode;
            mode |= static_cast<int>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_HIL_ENABLED);

            mavlinkcom::MavCmdDoSetMode cmd;
            cmd.command = static_cast<uint16_t>(mavlinkcom::MAV_CMD::MAV_CMD_DO_SET_MODE);
            cmd.Mode = static_cast<float>(mode);
            mav_vehicle_->sendCommand(cmd);

            is_hil_mode_set_ = true;
        }

        bool startOffboardMode()
        {
            checkValidVehicle();
            try {
                mav_vehicle_->requestControl();
            }
            catch (std::exception& ex) {
                ensureSafeMode();
                addStatusMessage(std::string("Request control failed: ") + ex.what());
                return false;
            }
            return true;
        }

        void endOffboardMode()
        {
            // bugbug: I removed this releaseControl because it makes back-to-back move operations less smooth.
            // The side effect of this is that with some drones (e.g. PX4 based) the drone itself will timeout
            // when you stop sending move commands and the behavior on timeout is then determined by the drone itself.
            // mav_vehicle_->releaseControl();
            ensureSafeMode();
        }

        void ensureSafeMode()
        {
            if (mav_vehicle_ != nullptr) {
                const mavlinkcom::VehicleState& state = mav_vehicle_->getVehicleState();
                if (state.controls.landed || !state.controls.armed) {
                    return;
                }
            }
        }

        void checkValidVehicle()
        {
            if (mav_vehicle_ == nullptr || connection_ == nullptr || !connection_->isOpen() || !connected_) {
                throw std::logic_error("Cannot perform operation when no vehicle is connected or vehicle is not responding");
            }
        }

        //status update methods should call this first!
        void updateState() const
        {
            StatusLock lock(this);
            if (mav_vehicle_ != nullptr) {
                int version = mav_vehicle_->getVehicleStateVersion();
                if (version != state_version_) {
                    current_state_ = mav_vehicle_->getVehicleState();
                    state_version_ = version;
                }
            }
        }

        virtual void normalizeRotorControls()
        {
            //if rotor controls are in not in 0-1 range then they are in -1 to 1 range in which case
            //we normalize them to 0 to 1 for PX4
            if (!is_controls_0_1_) {
                // change -1 to 1 to 0 to 1.
                for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                    rotor_controls_[i] = (rotor_controls_[i] + 1.0f) / 2.0f;
                }
            }
            else {
                //this applies to PX4 and may work differently on other firmwares.
                //We use 0.2 as idle rotors which leaves out range of 0.8
                for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                    rotor_controls_[i] = Utils::clip(0.8f * rotor_controls_[i] + 0.20f, 0.0f, 1.0f);
                }
            }
        }

        bool sendTestMessage(std::shared_ptr<mavlinkcom::MavLinkNode> node)
        {
            try {
                // try and send a test message.
                mavlinkcom::MavLinkHeartbeat test;
                test.autopilot = static_cast<int>(mavlinkcom::MAV_AUTOPILOT::MAV_AUTOPILOT_PX4);
                test.type = static_cast<uint8_t>(mavlinkcom::MAV_TYPE::MAV_TYPE_GCS);
                test.base_mode = 0;
                test.custom_mode = 0;
                test.mavlink_version = 3;
                node->sendMessage(test);
                test.system_status = 0;
                return true;
            }
            catch (std::exception&) {
                return false;
            }
        }

        bool connectToLogViewer()
        {
            //set up logviewer proxy
            if (connection_info_.logviewer_ip_address.size() > 0) {
                std::shared_ptr<mavlinkcom::MavLinkConnection> connection;
                createProxy("LogViewer", connection_info_.logviewer_ip_address, connection_info_.logviewer_ip_port, connection_info_.local_host_ip, logviewer_proxy_, connection);
                if (!sendTestMessage(logviewer_proxy_)) {
                    // error talking to log viewer, so don't keep trying, and close the connection also.
                    logviewer_proxy_->getConnection()->close();
                    logviewer_proxy_ = nullptr;
                }

                std::shared_ptr<mavlinkcom::MavLinkConnection> out_connection;
                createProxy("LogViewerOut", connection_info_.logviewer_ip_address, connection_info_.logviewer_ip_sport, connection_info_.local_host_ip, logviewer_out_proxy_, out_connection);
                if (!sendTestMessage(logviewer_out_proxy_)) {
                    // error talking to log viewer, so don't keep trying, and close the connection also.
                    logviewer_out_proxy_->getConnection()->close();
                    logviewer_out_proxy_ = nullptr;
                }
                else if (mav_vehicle_ != nullptr) {
                    auto proxylog = std::make_shared<MavLinkLogViewerLog>(logviewer_out_proxy_);
                    mav_vehicle_->getConnection()->startLoggingSendMessage(proxylog);
                    mav_vehicle_->getConnection()->startLoggingReceiveMessage(proxylog);
                    if (connection_ != nullptr) {
                        connection_->startLoggingSendMessage(proxylog);
                        connection_->startLoggingReceiveMessage(proxylog);
                    }
                }
            }
            return logviewer_proxy_ != nullptr;
        }

        bool connectToQGC()
        {
            if (connection_info_.qgc_ip_address.size() > 0) {
                std::shared_ptr<mavlinkcom::MavLinkConnection> connection;
                createProxy("QGC", connection_info_.qgc_ip_address, connection_info_.qgc_ip_port, connection_info_.local_host_ip, qgc_proxy_, connection);
                if (!sendTestMessage(qgc_proxy_)) {
                    // error talking to QGC, so don't keep trying, and close the connection also.
                    qgc_proxy_->getConnection()->close();
                    qgc_proxy_ = nullptr;
                }
                else {
                    connection->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection_val, const mavlinkcom::MavLinkMessage& msg) {
                        unused(connection_val);
                        processQgcMessages(msg);
                    });
                }
            }
            return qgc_proxy_ != nullptr;
        }

        void createProxy(std::string name, std::string ip, int port, string local_host_ip,
                         std::shared_ptr<mavlinkcom::MavLinkNode>& node, std::shared_ptr<mavlinkcom::MavLinkConnection>& connection)
        {
            if (connection_ == nullptr)
                throw std::domain_error("MavLinkMultirotorApi requires connection object to be set before createProxy call");

            connection = mavlinkcom::MavLinkConnection::connectRemoteUdp("Proxy to: " + name + " at " + ip + ":" + std::to_string(port), local_host_ip, ip, port);

            // it is ok to reuse the simulator sysid and compid here because this node is only used to send a few messages directly to this endpoint
            // and all other messages are funneled through from PX4 via the Join method below.
            node = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
            node->connect(connection);

            // now join the main connection to this one, this causes all PX4 messages to be sent to the proxy and all messages from the proxy will be
            // send directly to the PX4 (using whatever sysid/compid comes from that remote node).
            connection_->join(connection);

            auto mavcon = mav_vehicle_->getConnection();
            if (mavcon != connection_) {
                mavcon->join(connection);
            }
        }

        static std::string findPX4()
        {
            auto result = mavlinkcom::MavLinkConnection::findSerialPorts(0, 0);
            for (auto iter = result.begin(); iter != result.end(); iter++) {
                mavlinkcom::SerialPortInfo info = *iter;
                if ((
                        (info.vid == pixhawkVendorId) &&
                        (info.pid == pixhawkFMUV4ProductId || info.pid == pixhawkFMUV2ProductId || info.pid == pixhawkFMUV2OldBootloaderProductId || info.pid == pixhawkFMUV5ProductId)) ||
                    ((info.displayName.find(L"PX4_") != std::string::npos))) {
                    // printf("Auto Selecting COM port: %S\n", info.displayName.c_str());

                    std::string portName_str;

                    for (wchar_t ch : info.portName) {
                        portName_str.push_back(static_cast<char>(ch));
                    }
                    return portName_str;
                }
            }
            return "";
        }

        void createMavConnection(const AirSimSettings::MavLinkConnectionInfo& connection_info)
        {
            if (connection_info.use_serial) {
                createMavSerialConnection(connection_info.serial_port, connection_info.baud_rate);
            }
            else {
                createMavEthernetConnection(connection_info);
            }

            //Uncomment below for sending images over MavLink
            //connectToVideoServer();
        }

        void createMavEthernetConnection(const AirSimSettings::MavLinkConnectionInfo& connection_info)
        {
            close();

            connecting_ = true;
            is_controls_0_1_ = true;
            std::string remoteIpAddr;
            Utils::setValue(rotor_controls_, 0.0f);

            if (connection_info.use_tcp) {
                if (connection_info.tcp_port == 0) {
                    throw std::invalid_argument("TcpPort setting has an invalid value.");
                }

                auto msg = Utils::stringf("Waiting for TCP connection on port %d, local IP %s", connection_info.tcp_port, connection_info_.local_host_ip.c_str());
                addStatusMessage(msg);
                try {
                    connection_ = std::make_shared<mavlinkcom::MavLinkConnection>();
                    remoteIpAddr = connection_->acceptTcp("hil", connection_info_.local_host_ip, connection_info.tcp_port);
                }
                catch (std::exception& e) {
                    addStatusMessage("Accepting TCP socket failed, is another instance running?");
                    addStatusMessage(e.what());
                    return;
                }
            }
            else if (connection_info.udp_address.size() > 0) {
                if (connection_info.udp_port == 0) {
                    throw std::invalid_argument("UdpPort setting has an invalid value.");
                }

                connection_ = mavlinkcom::MavLinkConnection::connectRemoteUdp("hil", connection_info_.local_host_ip, connection_info.udp_address, connection_info.udp_port);
            }
            else {
                throw std::invalid_argument("Please provide valid connection info for your drone.");
            }

            // start listening to the SITL connection.
            connection_->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
                unused(connection);
                processMavMessages(msg);
            });

            hil_node_ = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
            hil_node_->connect(connection_);

            if (connection_info.use_tcp) {
                addStatusMessage(std::string("Connected to SITL over TCP."));
            }
            else {
                addStatusMessage(std::string("Connected to SITL over UDP."));
            }

            mav_vehicle_ = std::make_shared<mavlinkcom::MavLinkVehicle>(connection_info_.vehicle_sysid, connection_info_.vehicle_compid);

            if (connection_info_.control_ip_address != "") {
                if (connection_info_.control_port_local == 0) {
                    throw std::invalid_argument("LocalControlPort setting has an invalid value.");
                }
                if (!connection_info.use_tcp || connection_info_.control_ip_address != "remote") {
                    remoteIpAddr = connection_info_.control_ip_address;
                }
                if (remoteIpAddr == "local" || remoteIpAddr == "localhost") {
                    remoteIpAddr = "127.0.0.1";
                }

                // The PX4 SITL mode app cannot receive commands to control the drone over the same HIL mavlink connection.
                // The HIL mavlink connection can only handle HIL_SENSOR messages.  This separate channel is needed for
                // everything else.
                addStatusMessage(Utils::stringf("Connecting to PX4 Control UDP port %d, local IP %s, remote IP %s ...",
                                                connection_info_.control_port_local,
                                                connection_info_.local_host_ip.c_str(),
                                                remoteIpAddr.c_str()));

                // if we try and connect the UDP port too quickly it doesn't work, bug in PX4 ?
                for (int retries = 60; retries >= 0 && connecting_; retries--) {
                    try {
                        std::shared_ptr<mavlinkcom::MavLinkConnection> gcsConnection;
                        if (remoteIpAddr == "127.0.0.1") {
                            gcsConnection = mavlinkcom::MavLinkConnection::connectLocalUdp("gcs",
                                                                                           connection_info_.local_host_ip,
                                                                                           connection_info_.control_port_local);
                        }
                        else {
                            gcsConnection = mavlinkcom::MavLinkConnection::connectRemoteUdp("gcs",
                                                                                            connection_info_.local_host_ip,
                                                                                            remoteIpAddr,
                                                                                            connection_info_.control_port_remote);
                        }
                        mav_vehicle_->connect(gcsConnection);
                        // need to try and send something to make sure the connection is good.
                        mav_vehicle_->setMessageInterval(mavlinkcom::MavLinkHomePosition::kMessageId, 1);
                        break;
                    }
                    catch (std::exception&) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }

                if (mav_vehicle_ == nullptr) {
                    // play was stopped!
                    return;
                }

                if (mav_vehicle_->getConnection() != nullptr) {
                    addStatusMessage(std::string("Ground control connected over UDP."));
                }
                else {
                    addStatusMessage(std::string("Timeout trying to connect ground control over UDP."));
                    return;
                }
            }
            try {
                connectVehicle();
            }
            catch (std::exception& e) {
                addStatusMessage("Error connecting vehicle:");
                addStatusMessage(e.what());
            }
        }

        void processControlMessages(const mavlinkcom::MavLinkMessage& msg)
        {
            // Utils::log(Utils::stringf("Control msg %d", msg.msgid));
            // PX4 usually sends the following on the control channel.
            // If nothing is arriving here it means our control channel UDP connection isn't working.
            // MAVLINK_MSG_ID_HIGHRES_IMU
            // MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET
            // MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
            // MAVLINK_MSG_ID_GPS_RAW_INT
            // MAVLINK_MSG_ID_TIMESYNC
            // MAVLINK_MSG_ID_ALTITUDE
            // MAVLINK_MSG_ID_VFR_HUD
            // MAVLINK_MSG_ID_ESTIMATOR_STATUS
            // MAVLINK_MSG_ID_EXTENDED_SYS_STATE
            // MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
            // MAVLINK_MSG_ID_PING
            // MAVLINK_MSG_ID_SYS_STATUS
            // MAVLINK_MSG_ID_SYSTEM_TIME
            // MAVLINK_MSG_ID_LINK_NODE_STATUS
            // MAVLINK_MSG_ID_AUTOPILOT_VERSION
            // MAVLINK_MSG_ID_COMMAND_ACK
            // MAVLINK_MSG_ID_STATUSTEXT
            // MAVLINK_MSG_ID_PARAM_VALUE
            processMavMessages(msg);
        }

        void connectVehicle()
        {
            // listen to this UDP mavlink connection also
            auto mavcon = mav_vehicle_->getConnection();
            if (mavcon != nullptr && mavcon != connection_) {
                mavcon->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
                    unused(connection);
                    processControlMessages(msg);
                });
            }
            else {
                mav_vehicle_->connect(connection_);
            }

            connected_ = true;

            // Also request home position messages
            mav_vehicle_->setMessageInterval(mavlinkcom::MavLinkHomePosition::kMessageId, 1);

            // now we can start our heartbeats.
            mav_vehicle_->startHeartbeat();

            // wait for px4 to connect so we can safely send any configured parameters
            while (!send_params_ && connected_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (connected_) {
                sendParams();
                send_params_ = false;
            }
        }

        void createMavSerialConnection(const std::string& port_name, int baud_rate)
        {
            close();

            connecting_ = true;
            bool reported = false;
            std::string port_name_auto = port_name;
            while (port_name_auto == "" || port_name_auto == "*") {
                port_name_auto = findPX4();
                if (port_name_auto == "") {
                    if (!reported) {
                        reported = true;
                        addStatusMessage("Could not detect a connected PX4 flight controller on any USB ports.");
                        addStatusMessage("You can specify USB port in settings.json.");
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }

            if (port_name_auto == "") {
                addStatusMessage("USB port for PX4 flight controller is empty. Please set it in settings.json.");
                return;
            }

            if (baud_rate == 0) {
                addStatusMessage("Baud rate specified in settings.json is 0 which is invalid");
                return;
            }

            addStatusMessage(Utils::stringf("Connecting to PX4 over serial port: %s, baud rate %d ....", port_name_auto.c_str(), baud_rate));
            reported = false;

            while (connecting_) {
                try {
                    connection_ = mavlinkcom::MavLinkConnection::connectSerial("hil", port_name_auto, baud_rate);
                    connection_->ignoreMessage(mavlinkcom::MavLinkAttPosMocap::kMessageId); //TODO: find better way to communicate debug pose instead of using fake Mo-cap messages
                    hil_node_ = std::make_shared<mavlinkcom::MavLinkNode>(connection_info_.sim_sysid, connection_info_.sim_compid);
                    hil_node_->connect(connection_);
                    addStatusMessage(Utils::stringf("Connected to PX4 over serial port: %s", port_name_auto.c_str()));

                    // start listening to the HITL connection.
                    connection_->subscribe([=](std::shared_ptr<mavlinkcom::MavLinkConnection> connection, const mavlinkcom::MavLinkMessage& msg) {
                        unused(connection);
                        processMavMessages(msg);
                    });

                    mav_vehicle_ = std::make_shared<mavlinkcom::MavLinkVehicle>(connection_info_.vehicle_sysid, connection_info_.vehicle_compid);

                    connectVehicle();
                    return;
                }
                catch (std::exception& e) {
                    if (!reported) {
                        reported = true;
                        addStatusMessage("Error connecting to mavlink vehicle.");
                        addStatusMessage(e.what());
                        addStatusMessage("Please check your USB port in settings.json.");
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        mavlinkcom::MavLinkHilSensor getLastSensorMessage()
        {
            std::lock_guard<std::mutex> guard(last_message_mutex_);
            return last_sensor_message_;
        }

        mavlinkcom::MavLinkDistanceSensor getLastDistanceMessage()
        {
            std::lock_guard<std::mutex> guard(last_message_mutex_);
            return last_distance_message_;
        }

        mavlinkcom::MavLinkHilGps getLastGpsMessage()
        {
            std::lock_guard<std::mutex> guard(last_message_mutex_);
            return last_gps_message_;
        }

        void sendParams()
        {
            // send any mavlink parameters from settings.json through to the connected vehicle.
            if (mav_vehicle_ != nullptr && connection_info_.params.size() > 0) {
                try {
                    for (auto iter : connection_info_.params) {
                        auto key = iter.first;
                        auto value = iter.second;
                        mavlinkcom::MavLinkParameter p;
                        p.name = key;
                        p.value = value;
                        bool result = false;
                        mav_vehicle_->setParameter(p).wait(1000, &result);
                        if (!result) {
                            Utils::log(Utils::stringf("Failed to set mavlink parameter '%s'", key.c_str()));
                        }
                    }
                }
                catch (std::exception& ex) {
                    addStatusMessage("Exception sending parameters to vehicle");
                    addStatusMessage(ex.what());
                }
            }
        }

        void setArmed(bool armed)
        {
            if (is_armed_) {
                if (!armed) {
                    onDisarmed();
                }
            }
            else {
                if (armed) {
                    onArmed();
                }
            }

            is_armed_ = armed;
            if (!armed) {
                //reset motor controls
                for (size_t i = 0; i < Utils::length(rotor_controls_); ++i) {
                    rotor_controls_[i] = 0;
                }
            }
        }

        void processQgcMessages(const mavlinkcom::MavLinkMessage& msg)
        {
            if (msg.msgid == MocapPoseMessage.msgid) {
                std::lock_guard<std::mutex> guard(mocap_pose_mutex_);
                MocapPoseMessage.decode(msg);
                getMocapPose(mocap_pose_.position, mocap_pose_.orientation);
            }
            //else ignore message
        }

        void addStatusMessage(const std::string& message)
        {
            if (message.size() != 0) {
                Utils::log(message);
                std::lock_guard<std::mutex> guard_status(status_text_mutex_);
                //if queue became too large, clear it first
                if (status_messages_.size() > status_messages_MaxSize)
                    Utils::clear(status_messages_, status_messages_MaxSize - status_messages_.size());
                status_messages_.push(message);
            }
        }

        void handleLockStep()
        {
            received_actuator_controls_ = true;
            // if the timestamps match then it means we are in lockstep mode.
            if (!lock_step_active_ && lock_step_enabled_) {
                // && (HilActuatorControlsMessage.flags & 0x1))    // todo: enable this check when this flag is widely available...
                if (last_hil_sensor_time_ == HilActuatorControlsMessage.time_usec) {
                    addStatusMessage("Enabling lockstep mode");
                    lock_step_active_ = true;
                }
            }
            else {
                auto now = clock()->nowNanos() / 1000;
                auto delay = static_cast<uint32_t>(now - last_update_time_);

                std::lock_guard<std::mutex> guard(telemetry_mutex_);
                actuator_delay_ += delay;
            }
            if (world_ != nullptr) {
                world_->pause(false);
            }
        }

        void processMavMessages(const mavlinkcom::MavLinkMessage& msg)
        {
            if (msg.msgid == HeartbeatMessage.msgid) {
                std::lock_guard<std::mutex> guard_heartbeat(heartbeat_mutex_);

                HeartbeatMessage.decode(msg);

                bool armed = (HeartbeatMessage.base_mode & static_cast<uint8_t>(mavlinkcom::MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED)) > 0;
                setArmed(armed);
                if (!got_first_heartbeat_) {
                    Utils::log("received first heartbeat");

                    got_first_heartbeat_ = true;
                    if (HeartbeatMessage.autopilot == static_cast<uint8_t>(mavlinkcom::MAV_AUTOPILOT::MAV_AUTOPILOT_PX4) &&
                        HeartbeatMessage.type == static_cast<uint8_t>(mavlinkcom::MAV_TYPE::MAV_TYPE_FIXED_WING)) {
                        // PX4 will scale fixed wing servo outputs to -1 to 1
                        // and it scales multi rotor servo output to 0 to 1.
                        is_controls_0_1_ = false;
                    }
                }
                else if (is_simulation_mode_ && !is_hil_mode_set_) {
                    setHILMode();
                }
            }
            else if (msg.msgid == StatusTextMessage.msgid) {
                StatusTextMessage.decode(msg);
                //lock is established by below method
                addStatusMessage(std::string(StatusTextMessage.text));
            }
            else if (msg.msgid == CommandLongMessage.msgid) {
                CommandLongMessage.decode(msg);
                if (CommandLongMessage.command == static_cast<int>(mavlinkcom::MAV_CMD::MAV_CMD_SET_MESSAGE_INTERVAL)) {
                    int msg_id = static_cast<int>(CommandLongMessage.param1 + 0.5);
                    if (msg_id == 115) { //HIL_STATE_QUATERNION
                        hil_state_freq_ = static_cast<int>(CommandLongMessage.param2 + 0.5);
                    }
                }
            }
            else if (msg.msgid == HilControlsMessage.msgid) {
                if (!actuators_message_supported_) {
                    std::lock_guard<std::mutex> guard_controls(hil_controls_mutex_);

                    HilControlsMessage.decode(msg);
                    rotor_controls_[0] = HilControlsMessage.roll_ailerons;
                    rotor_controls_[1] = HilControlsMessage.pitch_elevator;
                    rotor_controls_[2] = HilControlsMessage.yaw_rudder;
                    rotor_controls_[3] = HilControlsMessage.throttle;
                    rotor_controls_[4] = HilControlsMessage.aux1;
                    rotor_controls_[5] = HilControlsMessage.aux2;
                    rotor_controls_[6] = HilControlsMessage.aux3;
                    rotor_controls_[7] = HilControlsMessage.aux4;

                    normalizeRotorControls();
                    handleLockStep();
                }
            }
            else if (msg.msgid == HilActuatorControlsMessage.msgid) {
                actuators_message_supported_ = true;

                std::lock_guard<std::mutex> guard_actuator(hil_controls_mutex_); //use same mutex as HIL_CONTROl

                HilActuatorControlsMessage.decode(msg);
                bool isarmed = (HilActuatorControlsMessage.mode & 128) != 0;
                for (auto i = 0; i < 8; ++i) {
                    if (isarmed) {
                        rotor_controls_[i] = HilActuatorControlsMessage.controls[i];
                    }
                    else {
                        rotor_controls_[i] = 0;
                    }
                }
                if (isarmed) {
                    normalizeRotorControls();
                }

                handleLockStep();
            }
            else if (msg.msgid == MavLinkGpsRawInt.msgid) {
                MavLinkGpsRawInt.decode(msg);
                auto fix_type = static_cast<mavlinkcom::GPS_FIX_TYPE>(MavLinkGpsRawInt.fix_type);
                auto locked = (fix_type != mavlinkcom::GPS_FIX_TYPE::GPS_FIX_TYPE_NO_GPS &&
                               fix_type != mavlinkcom::GPS_FIX_TYPE::GPS_FIX_TYPE_NO_FIX);
                if (locked && !has_gps_lock_) {
                    addStatusMessage("Got GPS lock");
                    has_gps_lock_ = true;
                }
                if (!has_home_ && current_state_.home.is_set) {
                    addStatusMessage("Got GPS Home Location");
                    has_home_ = true;
                }
                send_params_ = true;
            }
            else if (msg.msgid == mavlinkcom::MavLinkLocalPositionNed::kMessageId) {
                // we are getting position information... so we can use this to check the stability of the z coordinate before takeoff.
                if (current_state_.controls.landed) {
                    monitorGroundAltitude();
                }
            }
            else if (msg.msgid == mavlinkcom::MavLinkExtendedSysState::kMessageId) {
                // check landed state.
                getLandedState();
                send_params_ = true;
            }
            else if (msg.msgid == mavlinkcom::MavLinkHomePosition::kMessageId) {
                mavlinkcom::MavLinkHomePosition home;
                home.decode(msg);
                // this is a good time to send the params
                send_params_ = true;
            }
            else if (msg.msgid == mavlinkcom::MavLinkSysStatus::kMessageId) {
                // this is a good time to send the params
                send_params_ = true;
            }
            else if (msg.msgid == mavlinkcom::MavLinkAutopilotVersion::kMessageId) {
                // this is a good time to send the params
                send_params_ = true;
            }
            else {
                // creates too much log output, only do this when debugging this issue specifically.
                // Utils::log(Utils::stringf("Ignoring msg %d from %d,%d ", msg.msgid, msg.compid, msg.sysid));
            }
        }

        void sendHILSensor(const Vector3r& acceleration, const Vector3r& gyro, const Vector3r& mag, float abs_pressure, float pressure_alt)
        {
            if (!is_simulation_mode_)
                throw std::logic_error("Attempt to send simulated sensor messages while not in simulation mode");

            mavlinkcom::MavLinkHilSensor hil_sensor;
            hil_sensor.time_usec = last_hil_sensor_time_ = getSimTime();

            hil_sensor.xacc = acceleration.x();
            hil_sensor.yacc = acceleration.y();
            hil_sensor.zacc = acceleration.z();
            hil_sensor.fields_updated = 0b111; // Set accel bit fields

            hil_sensor.xgyro = gyro.x();
            hil_sensor.ygyro = gyro.y();
            hil_sensor.zgyro = gyro.z();

            hil_sensor.fields_updated |= 0b111000; // Set gyro bit fields

            hil_sensor.xmag = mag.x();
            hil_sensor.ymag = mag.y();
            hil_sensor.zmag = mag.z();

            hil_sensor.fields_updated |= 0b111000000; // Set mag bit fields

            hil_sensor.abs_pressure = abs_pressure;
            hil_sensor.pressure_alt = pressure_alt;

            hil_sensor.fields_updated |= 0b1101000000000; // Set baro bit fields

            //TODO: enable temperature? diff_pressure
            if (was_reset_) {
                hil_sensor.fields_updated = static_cast<uint32_t>(1 << 31);
            }

            if (hil_node_ != nullptr) {
                hil_node_->sendMessage(hil_sensor);
                received_actuator_controls_ = false;
                if (lock_step_active_ && world_ != nullptr) {
                    world_->pauseForTime(1); // 1 second delay max waiting for actuator controls.
                }
            }

            std::lock_guard<std::mutex> guard(last_message_mutex_);
            last_sensor_message_ = hil_sensor;
        }

        void sendSystemTime()
        {
            // SYSTEM TIME from host
            auto tu = getSimTime();
            uint32_t ms = (uint32_t)(tu / 1000);
            if (ms != last_sys_time_) {
                last_sys_time_ = ms;
                mavlinkcom::MavLinkSystemTime msg_system_time;
                msg_system_time.time_unix_usec = tu;
                msg_system_time.time_boot_ms = last_sys_time_;
                if (hil_node_ != nullptr) {
                    hil_node_->sendMessage(msg_system_time);
                }
            }
        }

        void sendDistanceSensor(float min_distance, float max_distance, float current_distance,
                                uint8_t sensor_type, uint8_t sensor_id, const Quaternionr& orientation)
        {
            if (!is_simulation_mode_)
                throw std::logic_error("Attempt to send simulated distance sensor messages while not in simulation mode");

            mavlinkcom::MavLinkDistanceSensor distance_sensor;
            distance_sensor.time_boot_ms = static_cast<uint32_t>(getSimTime() / 1000);
            distance_sensor.min_distance = static_cast<uint16_t>(min_distance);
            distance_sensor.max_distance = static_cast<uint16_t>(max_distance);
            distance_sensor.current_distance = static_cast<uint16_t>(current_distance);
            distance_sensor.type = sensor_type;
            distance_sensor.id = sensor_id;

            // Use custom orientation
            distance_sensor.orientation = 100; // MAV_SENSOR_ROTATION_CUSTOM
            distance_sensor.quaternion[0] = orientation.w();
            distance_sensor.quaternion[1] = orientation.x();
            distance_sensor.quaternion[2] = orientation.y();
            distance_sensor.quaternion[3] = orientation.z();

            //TODO: use covariance parameter?
            // PX4 doesn't support receiving simulated distance sensor messages this way.
            // It results in lots of error messages from PX4.  This code is still useful in that
            // it sets last_distance_message_ and that is returned via Python API.
            //
            // if (hil_node_ != nullptr) {
            //    hil_node_->sendMessage(distance_sensor);
            // }

            std::lock_guard<std::mutex> guard(last_message_mutex_);
            last_distance_message_ = distance_sensor;
        }

        void sendHILGps(const GeoPoint& geo_point, const Vector3r& velocity, float velocity_xy, float cog,
                        float eph, float epv, int fix_type, unsigned int satellites_visible)
        {
            unused(satellites_visible);

            if (!is_simulation_mode_)
                throw std::logic_error("Attempt to send simulated GPS messages while not in simulation mode");

            mavlinkcom::MavLinkHilGps hil_gps;
            hil_gps.time_usec = getSimTime();
            hil_gps.lat = static_cast<int32_t>(geo_point.latitude * 1E7);
            hil_gps.lon = static_cast<int32_t>(geo_point.longitude * 1E7);
            hil_gps.alt = static_cast<int32_t>(geo_point.altitude * 1000);
            hil_gps.vn = static_cast<int16_t>(velocity.x() * 100);
            hil_gps.ve = static_cast<int16_t>(velocity.y() * 100);
            hil_gps.vd = static_cast<int16_t>(velocity.z() * 100);
            hil_gps.eph = static_cast<uint16_t>(eph * 100);
            hil_gps.epv = static_cast<uint16_t>(epv * 100);
            hil_gps.fix_type = static_cast<uint8_t>(fix_type);
            hil_gps.vel = static_cast<uint16_t>(velocity_xy * 100);
            hil_gps.cog = static_cast<uint16_t>(cog * 100);
            hil_gps.satellites_visible = static_cast<uint8_t>(15);

            if (hil_node_ != nullptr) {
                hil_node_->sendMessage(hil_gps);
            }

            if (hil_gps.lat < 0.1f && hil_gps.lat > -0.1f) {
                //Utils::DebugBreak();
                Utils::log("hil_gps.lat was too close to 0", Utils::kLogLevelError);
            }

            std::lock_guard<std::mutex> guard(last_message_mutex_);
            last_gps_message_ = hil_gps;
        }

        void resetState()
        {
            //reset state
            is_hil_mode_set_ = false;
            is_controls_0_1_ = true;
            hil_state_freq_ = -1;
            actuators_message_supported_ = false;
            state_version_ = 0;
            current_state_ = mavlinkcom::VehicleState();
            target_height_ = 0;
            got_first_heartbeat_ = false;
            is_armed_ = false;
            has_home_ = false;
            sim_time_us_ = 0;
            last_sys_time_ = 0;
            last_gps_time_ = 0;
            last_update_time_ = 0;
            last_hil_sensor_time_ = 0;
            update_count_ = 0;
            hil_sensor_count_ = 0;
            lock_step_resets_ = 0;
            actuator_delay_ = 0;
            is_api_control_enabled_ = false;
            thrust_controller_ = PidController();
            Utils::setValue(rotor_controls_, 0.0f);
            was_reset_ = false;
            received_actuator_controls_ = false;
            lock_step_active_ = false;
            lock_step_enabled_ = false;
            has_gps_lock_ = false;
            send_params_ = false;
            mocap_pose_ = Pose::nanPose();
            ground_variance_ = 1;
            ground_filter_.initialize(25, 0.1f);
            cancelLastTask();
        }

        void monitorGroundAltitude()
        {
            // used to ensure stable altitude before takeoff.
            auto position = getPosition();
            auto result = ground_filter_.filter(position.z());
            auto variance = std::get<1>(result);
            if (variance >= 0) { // filter returns -1 if we don't have enough data yet.
                ground_variance_ = variance;
            }
        }

    protected: //variables
        //TODO: below was made protected from private to support Ardupilot
        //implementation but we need to review this and avoid having protected variables
        static const int RotorControlsCount = 8;

        const SensorCollection* sensors_;
        mutable std::mutex hil_controls_mutex_;
        AirSimSettings::MavLinkConnectionInfo connection_info_;
        float rotor_controls_[RotorControlsCount];
        bool is_simulation_mode_;

    private: //variables
        static const int pixhawkVendorId = 9900; ///< Vendor ID for Pixhawk board (V2 and V1) and PX4 Flow
        static const int pixhawkFMUV4ProductId = 18; ///< Product ID for Pixhawk V2 board
        static const int pixhawkFMUV2ProductId = 17; ///< Product ID for Pixhawk V2 board
        static const int pixhawkFMUV5ProductId = 50; ///< Product ID for Pixhawk V5 board
        static const int pixhawkFMUV2OldBootloaderProductId = 22; ///< Product ID for Bootloader on older Pixhawk V2 boards
        static const int pixhawkFMUV1ProductId = 16; ///< Product ID for PX4 FMU V1 board
        static const int messageReceivedTimeout = 10; ///< Seconds

        std::shared_ptr<mavlinkcom::MavLinkNode> logviewer_proxy_, logviewer_out_proxy_, qgc_proxy_;

        size_t status_messages_MaxSize = 5000;

        std::shared_ptr<mavlinkcom::MavLinkNode> hil_node_;
        std::shared_ptr<mavlinkcom::MavLinkConnection> connection_;
        std::shared_ptr<mavlinkcom::MavLinkVideoServer> video_server_;
        std::shared_ptr<MultirotorApiBase> mav_vehicle_control_;

        mavlinkcom::MavLinkAttPosMocap MocapPoseMessage;
        mavlinkcom::MavLinkHeartbeat HeartbeatMessage;
        mavlinkcom::MavLinkSetMode SetModeMessage;
        mavlinkcom::MavLinkStatustext StatusTextMessage;
        mavlinkcom::MavLinkHilControls HilControlsMessage;
        mavlinkcom::MavLinkHilActuatorControls HilActuatorControlsMessage;
        mavlinkcom::MavLinkGpsRawInt MavLinkGpsRawInt;
        mavlinkcom::MavLinkCommandLong CommandLongMessage;
        mavlinkcom::MavLinkLocalPositionNed MavLinkLocalPositionNed;

        mavlinkcom::MavLinkHilSensor last_sensor_message_;
        mavlinkcom::MavLinkDistanceSensor last_distance_message_;
        mavlinkcom::MavLinkHilGps last_gps_message_;

        std::mutex mocap_pose_mutex_, heartbeat_mutex_, set_mode_mutex_, status_text_mutex_, last_message_mutex_, telemetry_mutex_;

        //variables required for VehicleApiBase implementation
        bool got_first_heartbeat_ = false, is_hil_mode_set_ = false, is_armed_ = false;
        bool is_controls_0_1_; //Are motor controls specified in 0..1 or -1..1?
        bool send_params_ = false;
        std::queue<std::string> status_messages_;
        int hil_state_freq_;
        bool actuators_message_supported_ = false;
        bool was_reset_ = false;
        bool has_home_ = false;
        bool is_ready_ = false;
        bool has_gps_lock_ = false;
        bool lock_step_active_ = false;
        bool lock_step_enabled_ = false;
        bool received_actuator_controls_ = false;
        std::string is_ready_message_;
        Pose mocap_pose_;
        std::thread connect_thread_;
        bool connecting_ = false;
        bool connected_ = false;
        common_utils::SmoothingFilter<float> ground_filter_;
        double ground_variance_ = 1;
        const double GroundTolerance = 0.1;

        // variables for throttling HIL_SENSOR and SYSTEM_TIME messages
        uint32_t last_sys_time_ = 0;
        unsigned long long sim_time_us_ = 0;
        uint64_t last_gps_time_ = 0;
        uint64_t last_update_time_ = 0;
        uint64_t last_hil_sensor_time_ = 0;

        // variables accumulated for MavLinkTelemetry messages.
        uint64_t update_time_ = 0;
        uint32_t update_count_ = 0;
        uint32_t hil_sensor_count_ = 0;
        uint32_t lock_step_resets_ = 0;
        uint32_t actuator_delay_ = 0;
        std::thread telemetry_thread_;

        //additional variables required for MultirotorApiBase implementation
        //this is optional for methods that might not use vehicle commands
        std::shared_ptr<mavlinkcom::MavLinkVehicle> mav_vehicle_;
        float target_height_;
        bool is_api_control_enabled_;
        PidController thrust_controller_;
        common_utils::Timer hil_message_timer_;
        common_utils::Timer gcs_message_timer_;
        std::shared_ptr<mavlinkcom::MavLinkFileLog> log_;
        std::string log_file_name_;
        World* world_;

        //every time we return status update, we need to check if we have new data
        //this is why below two variables are marked as mutable
        mutable int state_version_;
        mutable mavlinkcom::VehicleState current_state_;
    };
}
} //namespace
#endif

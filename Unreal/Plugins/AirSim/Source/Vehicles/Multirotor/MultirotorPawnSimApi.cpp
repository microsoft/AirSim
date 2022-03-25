#include "MultirotorPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(const Params& params)
    : PawnSimApi(params), pawn_events_(static_cast<MultirotorPawnEvents*>(params.pawn_events))
{
}

void MultirotorPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();
    //setup physics vehicle
    multirotor_physics_body_ = std::unique_ptr<MultiRotor>(new MultiRotorPhysicsBody(vehicle_params_.get(), vehicle_api_.get(), getKinematics(), getEnvironment()));
    rotor_count_ = multirotor_physics_body_->wrenchVertexCount();
    rotor_actuator_info_.assign(rotor_count_, RotorActuatorInfo());

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
    rotor_states_.rotors.assign(rotor_count_, RotorParameters());

    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);
}

void MultirotorPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void MultirotorPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    multirotor_physics_body_->setCollisionInfo(collision_info);

    last_phys_pose_ = multirotor_physics_body_->getPose();

    collision_response = multirotor_physics_body_->getCollisionResponseInfo();

    //update rotor poses
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rotor_output = multirotor_physics_body_->getRotorOutput(i);
        // update private rotor variable
        rotor_states_.rotors[i].update(rotor_output.thrust, rotor_output.torque_scaler, rotor_output.speed);
        RotorActuatorInfo* info = &rotor_actuator_info_[i];
        info->rotor_speed = rotor_output.speed;
        info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
        info->rotor_thrust = rotor_output.thrust;
        info->rotor_control_filtered = rotor_output.control_signal_filtered;
    }

    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
    rotor_states_.timestamp = clock()->nowNanos();
    vehicle_api_->setRotorStates(rotor_states_);
}

void MultirotorPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ == PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"),
                                 FString::FromInt(collision_response.collision_count_non_resting),
                                 LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception& e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    pawn_events_->getActuatorSignal().emit(rotor_actuator_info_);
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    multirotor_physics_body_->lock();
    multirotor_physics_body_->setPose(pose);
    multirotor_physics_body_->setGrounded(false);
    multirotor_physics_body_->unlock();
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

void MultirotorPawnSimApi::setKinematics(const Kinematics::State& state, bool ignore_collision)
{
    multirotor_physics_body_->lock();
    multirotor_physics_body_->updateKinematics(state);
    multirotor_physics_body_->setGrounded(false);
    multirotor_physics_body_->unlock();
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    multirotor_physics_body_->reset();
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void MultirotorPawnSimApi::update()
{
    //environment update for current position
    PawnSimApi::update();

    //update forces on vertices
    multirotor_physics_body_->update();

    //update to controller must be done after kinematics have been updated by physics engine
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    multirotor_physics_body_->reportState(reporter);
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
    return multirotor_physics_body_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

std::string MultirotorPawnSimApi::getRecordFileLine(bool is_header_line) const
{
    std::ostringstream ss;
    if (getVehicleSetting()->vehicle_type == "" || //default config
        getVehicleSetting()->vehicle_type == msr::airlib::AirSimSettings::kVehicleTypeSimpleFlight) {

        std::string common_line = PawnSimApi::getRecordFileLine(is_header_line);
        if (is_header_line) {
            return common_line +
                   "TRUE_POS_X\tTRUE_POS_Y\tTRUE_POS_Z\tTRUE_VEL_X\tTRUE_VEL_Y\tTRUE_VEL_Z\tTRUE_Q_W\tTRUE_Q_X\tTRUE_Q_Y\tTRUE_Q_Z\tTRUE_ANGLE_ROLL\tTRUE_ANGLE_PITCH\tTRUE_ANGLE_YAW\t" +
                   "EKF_POS_X\tEKF_POS_Y\tEKF_POS_Z\tEKF_VEL_X\tEKF_VEL_Y\tEKF_VEL_Z\tEKF_Q_W\tEKF_Q_X\tEKF_Q_Y\tEKF_Q_Z\tEKF_ANGLE_ROLL\tEKF_ANGLE_PITCH\tEKF_ANGLE_YAW\t" +
                   "EKF_POS_X_VAR\tEKF_POS_Y_VAR\tEKF_POS_Z_VAR\tEKF_VEL_X_VAR\tEKF_VEL_Y_VAR\tEKF_VEL_Z_VAR\tEKF_Q_W_VAR\tEKF_Q_X_VAR\tEKF_Q_Y_VAR\tEKF_Q_Z_VAR\tEKF_ANGLE_ROLL_VAR\tEKF_ANGLE_PITCH_VAR\tEKF_ANGLE_YAW_VAR\t" +
                   "QUAT_NORM\t" +
                   "ACCEL_X\tACCEL_Y\tACCEL_Z\t" +
                   "GYRO_X\tGYRO_Y\tGYRO_Z\t" +
                   "GPS_POS_X\tGPS_POS_Y\tGPS_POS_Z\t" +
                   "GPS_VEL_X\tGPS_VEL_Y\tGPS_VEL_Z\t" +
                   "BARO_ALT\t" +
                   "MAG_X\tMAG_Y\tMAG_Z\t" +
                   "BIAS_ACCEL_X\tBIAS_ACCEL_Y\tBIAS_ACCEL_Z\t" +
                   "BIAS_GYRO_X\tBIAS_GYRO_Y\tBIAS_GYRO_Z\t" +
                   "BIAS_BARO\t" +
                   "BIAS_ACCEL_X_VAR\tBIAS_ACCEL_Y_VAR\tBIAS_ACCEL_Z_VAR\t" +
                   "BIAS_GYRO_X_VAR\tBIAS_GYRO_Y_VAR\tBIAS_GYRO_Z_VAR\t" +
                   "BIAS_BARO_VAR\t";
        }

        const SimpleFlightApi* simple_flight_vehicle_api = static_cast<const SimpleFlightApi*>(getVehicleApi());
        const auto& true_kin_state = simple_flight_vehicle_api->getTrueKinematicsEstimated();
        const auto& true_angles = simple_flight_vehicle_api->getTrueAngles();
        const auto& ekf_state = simple_flight_vehicle_api->getEkfKinematicsEstimated();
        const auto& ekf_state_variance = simple_flight_vehicle_api->getEkfStateVariance();
        const auto& ekf_meas = simple_flight_vehicle_api->getEkfMeasurements();
        const auto& quaternion_norm = simple_flight_vehicle_api->getEkfOrientationNorm();

        ss << common_line;

        ss << true_kin_state.pose.position.x() << "\t" << true_kin_state.pose.position.y() << "\t" << true_kin_state.pose.position.z() << "\t";
        ss << true_kin_state.twist.linear.x() << "\t" << true_kin_state.twist.linear.y() << "\t" << true_kin_state.twist.linear.z() << "\t";
        ss << true_kin_state.pose.orientation.w() << '\t' << true_kin_state.pose.orientation.x() << "\t" << true_kin_state.pose.orientation.y() << "\t" << true_kin_state.pose.orientation.z() << "\t";
        ss << true_angles.x() << "\t" << true_angles.y() << "\t" << true_angles.z() << "\t";

        ss << ekf_state.position.x() << "\t" << ekf_state.position.y() << "\t" << ekf_state.position.z() << "\t";
        ss << ekf_state.linear_velocity.x() << "\t" << ekf_state.linear_velocity.y() << "\t" << ekf_state.linear_velocity.z() << "\t";
        ss << ekf_state.orientation.w() << '\t' << ekf_state.orientation.x() << "\t" << ekf_state.orientation.y() << "\t" << ekf_state.orientation.z() << "\t";
        ss << ekf_state.angles.x() << "\t" << ekf_state.angles.y() << "\t" << ekf_state.angles.z() << "\t";

        ss << ekf_state_variance.position.x() << "\t" << ekf_state_variance.position.y() << "\t" << ekf_state_variance.position.z() << "\t";
        ss << ekf_state_variance.linear_velocity.x() << "\t" << ekf_state_variance.linear_velocity.y() << "\t" << ekf_state_variance.linear_velocity.z() << "\t";
        ss << ekf_state_variance.orientation.w() << '\t' << ekf_state_variance.orientation.x() << "\t" << ekf_state_variance.orientation.y() << "\t" << ekf_state_variance.orientation.z() << "\t";
        ss << ekf_state_variance.angles.x() << "\t" << ekf_state_variance.angles.y() << "\t" << ekf_state_variance.angles.z() << "\t";

        ss << quaternion_norm << "\t";

        ss << ekf_meas.accel.x() << "\t" << ekf_meas.accel.y() << "\t" << ekf_meas.accel.z() << "\t";
        ss << ekf_meas.gyro.x() << "\t" << ekf_meas.gyro.y() << "\t" << ekf_meas.gyro.z() << "\t";
        ss << ekf_meas.gps_position.x() << "\t" << ekf_meas.gps_position.y() << "\t" << ekf_meas.gps_position.z() << "\t";
        ss << ekf_meas.gps_velocity.x() << "\t" << ekf_meas.gps_velocity.y() << "\t" << ekf_meas.gps_velocity.z() << "\t";
        ss << ekf_meas.baro_altitude << "\t";
        ss << ekf_meas.magnetic_flux.x() << "\t" << ekf_meas.magnetic_flux.y() << "\t" << ekf_meas.magnetic_flux.z() << "\t";

        ss << ekf_state.sensor_bias.accel.x() << "\t" << ekf_state.sensor_bias.accel.y() << "\t" << ekf_state.sensor_bias.accel.z() << "\t";
        ss << ekf_state.sensor_bias.gyro.x() << "\t" << ekf_state.sensor_bias.gyro.y() << "\t" << ekf_state.sensor_bias.gyro.z() << "\t";
        ss << ekf_state.sensor_bias.barometer << "\t";

        ss << ekf_state_variance.sensor_bias.accel.x() << "\t" << ekf_state_variance.sensor_bias.accel.y() << "\t" << ekf_state_variance.sensor_bias.accel.z() << "\t";
        ss << ekf_state_variance.sensor_bias.gyro.x() << "\t" << ekf_state_variance.sensor_bias.gyro.y() << "\t" << ekf_state_variance.sensor_bias.gyro.z() << "\t";
        ss << ekf_state_variance.sensor_bias.barometer << "\t";
    }

    return ss.str();
}
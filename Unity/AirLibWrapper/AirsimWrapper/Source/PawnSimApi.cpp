#include "PawnSimApi.h"
#include "common/ClockFactory.hpp"
#include "common/EarthUtils.hpp"
#include "PInvokeWrapper.h"
#include "AirSimStructs.hpp"
#include "UnityUtilities.hpp"

PawnSimApi::PawnSimApi(const Params& params)
	: params_(params), ned_transform_(*params.global_transform)
{
	image_capture_.reset(new UnityImageCapture(params.vehicle_name));

	msr::airlib::Environment::State initial_environment;
	initial_environment.position = getPose().position;
	initial_environment.geo_point = params_.home_geopoint;
	environment_.reset(new msr::airlib::Environment(initial_environment));

	//initialize state
	UnityTransform initialTransform = GetTransformFromUnity(params.vehicle_name.c_str());
	initial_state_.mesh_origin = initialTransform.Position;
	setStartPosition(initialTransform.Position, initialTransform.Rotation);

	initial_state_.tracing_enabled = getVehicleSetting()->enable_trace;
	initial_state_.collisions_enabled = getVehicleSetting()->enable_collisions;
	initial_state_.passthrough_enabled = getVehicleSetting()->enable_collision_passthrough;

	initial_state_.collision_info = CollisionInfo();

	initial_state_.was_last_move_teleport = false;
	initial_state_.was_last_move_teleport = canTeleportWhileMove();
	state_ = initial_state_;
}

void PawnSimApi::setStartPosition(const AirSimVector& position, const AirSimQuaternion& rotator)
{
	initial_state_.start_location = position;
	initial_state_.start_rotation = rotator;

	//compute our home point
	home_geo_point_ = msr::airlib::EarthUtils::nedToGeodetic(UnityUtilities::Convert_to_Vector3r(initial_state_.start_location),
		AirSimSettings::singleton().origin_geopoint);
}

void PawnSimApi::pawnTick(float dt)
{
	update();
	updateRenderedState(dt);
	updateRendering(dt);
}

void PawnSimApi::OnCollision(msr::airlib::CollisionInfo collisionInfo)
{
	state_.collision_info.has_collided = collisionInfo.has_collided;
	state_.collision_info.normal = collisionInfo.normal;
	state_.collision_info.impact_point = collisionInfo.impact_point;
	state_.collision_info.position = collisionInfo.position;
	state_.collision_info.penetration_depth = collisionInfo.penetration_depth;
	state_.collision_info.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
	state_.collision_info.object_name = collisionInfo.object_name;
	state_.collision_info.object_id = collisionInfo.object_id;

	++state_.collision_info.collision_count;

	PrintLogMessage("Collision", Utils::stringf("#%d with %s - ObjID %d",
		state_.collision_info.collision_count,
		state_.collision_info.object_name.c_str(), state_.collision_info.object_id).c_str(), getVehicleName().c_str(), ErrorLogSeverity::Information);
}

const NedTransform& PawnSimApi::getNedTransform() const
{
	return ned_transform_;
}

std::vector<PawnSimApi::ImageCaptureBase::ImageResponse> PawnSimApi::getImages(
	const std::vector<ImageCaptureBase::ImageRequest>& requests) const
{
	std::vector<ImageCaptureBase::ImageResponse> responses;
	const ImageCaptureBase* camera = getImageCapture();
	camera->getImages(requests, responses);
	return responses;
}

std::vector<uint8_t> PawnSimApi::getImage(const std::string& camera_name, ImageCaptureBase::ImageType image_type) const
{
	std::vector<ImageCaptureBase::ImageRequest> request = { ImageCaptureBase::ImageRequest(camera_name, image_type) };
	const std::vector<ImageCaptureBase::ImageResponse>& response = getImages(request);
	if (response.size() > 0)
		return response.at(0).image_data_uint8;
	else
		return std::vector<uint8_t>();
}

msr::airlib::RCData PawnSimApi::getRCData() const
{
	AirSimRCData rcDataFromUnity = GetRCData(getVehicleName().c_str());
	rc_data_.is_valid = rcDataFromUnity.is_valid;

	if (rc_data_.is_valid)
	{
		rc_data_.throttle = rcDataFromUnity.throttle;
		rc_data_.yaw = rcDataFromUnity.yaw;
		rc_data_.roll = rcDataFromUnity.roll;
		rc_data_.pitch = rcDataFromUnity.pitch;

		//these will be available for devices like steering wheels
		rc_data_.left_z = rcDataFromUnity.left_z;
		rc_data_.right_z = rcDataFromUnity.right_z;

		rc_data_.switches = 0;
		rc_data_.switches |= ((rcDataFromUnity.switch1 & 0x80) == 0 ? 0 : 1) << 0;
		rc_data_.switches |= ((rcDataFromUnity.switch2 & 0x80) == 0 ? 0 : 1) << 1;
		rc_data_.switches |= ((rcDataFromUnity.switch3 & 0x80) == 0 ? 0 : 1) << 2;
		rc_data_.switches |= ((rcDataFromUnity.switch4 & 0x80) == 0 ? 0 : 1) << 3;
		rc_data_.switches |= ((rcDataFromUnity.switch5 & 0x80) == 0 ? 0 : 1) << 4;
		rc_data_.switches |= ((rcDataFromUnity.switch6 & 0x80) == 0 ? 0 : 1) << 5;
		rc_data_.switches |= ((rcDataFromUnity.switch7 & 0x80) == 0 ? 0 : 1) << 6;
		rc_data_.switches |= ((rcDataFromUnity.switch8 & 0x80) == 0 ? 0 : 1) << 7;
		PrintLogMessage("RC Mode: ", rc_data_.getSwitch(0) == 0 ? "Angle" : "Rate", getVehicleName().c_str(), ErrorLogSeverity::Information);
	}

	return rc_data_;
}

int PawnSimApi::getRemoteControlID() const
{
	return getVehicleSetting()->rc.remote_control_id;
}

const UnityImageCapture* PawnSimApi::getImageCapture() const
{
	return image_capture_.get();
}

AirSimPose PawnSimApi::GetInitialPose()
{
	return AirSimPose(state_.start_location, state_.start_rotation);
}

void PawnSimApi::reset()
{
	VehicleSimApiBase::reset();
	state_ = initial_state_;
	rc_data_ = msr::airlib::RCData();
	environment_->reset();
}

void PawnSimApi::update()
{
	//update position from kinematics so we have latest position after physics update
	environment_->setPosition(kinematics_.pose.position);
	environment_->update();
	//kinematics_->update();

	VehicleSimApiBase::update();
}

PawnSimApi::CollisionInfo PawnSimApi::getCollisionInfo() const
{
	return state_.collision_info;
}

void PawnSimApi::toggleTrace()
{
	state_.tracing_enabled = !state_.tracing_enabled;
}

void PawnSimApi::allowPassthroughToggleInput()
{
	state_.passthrough_enabled = !state_.passthrough_enabled;
	PrintLogMessage("enable_passthrough_on_collisions: ", state_.passthrough_enabled ? "true" : "false", params_.vehicle_name.c_str(), ErrorLogSeverity::Information);
}

msr::airlib::CameraInfo PawnSimApi::getCameraInfo(const std::string& camera_name) const
{
	AirSimCameraInfo airsim_camera_info = GetCameraInfo(camera_name.c_str(), params_.vehicle_name.c_str()); // Into Unity
	msr::airlib::CameraInfo camera_info;
	camera_info.pose.position.x() = airsim_camera_info.pose.position.x;
	camera_info.pose.position.y() = airsim_camera_info.pose.position.y;
	camera_info.pose.position.z() = airsim_camera_info.pose.position.z;
	camera_info.pose.orientation.x() = airsim_camera_info.pose.orientation.x;
	camera_info.pose.orientation.y() = airsim_camera_info.pose.orientation.y;
	camera_info.pose.orientation.z() = airsim_camera_info.pose.orientation.z;
	camera_info.pose.orientation.w() = airsim_camera_info.pose.orientation.w;
	camera_info.fov = airsim_camera_info.fov;
	return camera_info;
}

void PawnSimApi::setCameraOrientation(const std::string& camera_name, const msr::airlib::Quaternionr& orientation)
{
	AirSimQuaternion airSimOrientation;
	airSimOrientation.x = orientation.x();
	airSimOrientation.y = orientation.y();
	airSimOrientation.z = orientation.z();
	airSimOrientation.w = orientation.w();

	SetCameraOrientation(camera_name.c_str(), airSimOrientation, params_.vehicle_name.c_str());
}

//parameters in NED frame
PawnSimApi::Pose PawnSimApi::getPose() const
{
	AirSimUnity::AirSimPose airSimPose = GetPose(params_.vehicle_name.c_str());
	return UnityUtilities::Convert_to_Pose(airSimPose);
}

void PawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
	SetPose(UnityUtilities::Convert_to_AirSimPose(pose), ignore_collision, getVehicleName().c_str());
}

bool PawnSimApi::canTeleportWhileMove()  const
{
	//allow teleportation
	//  if collisions are not enabled
	//  or we have collided but passthrough is enabled
	//     we will flip-flop was_last_move_teleport flag so on one tick we have passthrough and other tick we don't
	//     without flip flopping, collisions can't be detected
	return !state_.collisions_enabled || (state_.collision_info.has_collided && !state_.was_last_move_teleport && state_.passthrough_enabled);
}

const msr::airlib::Kinematics::State* PawnSimApi::getPawnKinematics() const
{
	return &kinematics_;
}

void PawnSimApi::updateRenderedState(float dt)
{
	updateKinematics(dt);
}

void PawnSimApi::updateRendering(float dt)
{
	unused(dt);
	//no default action in this base class
}

const msr::airlib::Kinematics::State* PawnSimApi::getGroundTruthKinematics() const
{
	return &kinematics_;
}

const msr::airlib::Environment* PawnSimApi::getGroundTruthEnvironment() const
{
	return environment_.get();
}

std::string PawnSimApi::getRecordFileLine(bool is_header_line) const
{
	if (is_header_line) 
	{
		return "TimeStamp\tPOS_X\tPOS_Y\tPOS_Z\tQ_W\tQ_X\tQ_Y\tQ_Z\t";
	}

	const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();
	uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

	//TODO: because this bug we are using alternative code with stringstream
	//https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html

	std::string line;
	line.append(std::to_string(timestamp_millis)).append("\t")
		.append(std::to_string(kinematics->pose.position.x())).append("\t")
		.append(std::to_string(kinematics->pose.position.y())).append("\t")
		.append(std::to_string(kinematics->pose.position.z())).append("\t")
		.append(std::to_string(kinematics->pose.orientation.w())).append("\t")
		.append(std::to_string(kinematics->pose.orientation.x())).append("\t")
		.append(std::to_string(kinematics->pose.orientation.y())).append("\t")
		.append(std::to_string(kinematics->pose.orientation.z())).append("\t")
		;

	return line;
}

void PawnSimApi::updateKinematics(float dt)
{
	const auto last_kinematics = kinematics_;

	kinematics_.pose = getPose();

	kinematics_.twist.linear = UnityUtilities::Convert_to_Vector3r(GetVelocity(getVehicleName().c_str()));
	kinematics_.twist.angular = msr::airlib::VectorMath::toAngularVelocity(
		kinematics_.pose.orientation, last_kinematics.pose.orientation, dt);

	kinematics_.accelerations.linear = (kinematics_.twist.linear - last_kinematics.twist.linear) / dt;
	kinematics_.accelerations.angular = (kinematics_.twist.angular - last_kinematics.twist.angular) / dt;
}
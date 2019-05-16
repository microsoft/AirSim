
#include "WorldSimApi.h"
#include "PInvokeWrapper.h"
#include "UnityUtilities.hpp"

WorldSimApi::WorldSimApi(SimModeBase* simmode, std::string vehicle_name)
	: simmode_(simmode), vehicle_name_(vehicle_name)
{}

WorldSimApi::~WorldSimApi()
{}

bool WorldSimApi::isPaused() const
{
	return simmode_->isPaused();
}

void WorldSimApi::reset()
{
	simmode_->reset();
}

void WorldSimApi::pause(bool is_paused)
{
	simmode_->pause(is_paused);
}

void WorldSimApi::continueForTime(double seconds)
{
	simmode_->continueForTime(seconds);
}

void WorldSimApi::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
    float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    simmode_->setTimeOfDay(is_enabled, start_datetime, is_start_datetime_dst,
        celestial_clock_speed, update_interval_secs, move_sun);
}

bool WorldSimApi::setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
	return SetSegmentationObjectId(mesh_name.c_str(), object_id, is_name_regex);
}

int WorldSimApi::getSegmentationObjectID(const std::string& mesh_name) const
{
	return GetSegmentationObjectId(mesh_name.c_str());
}

void WorldSimApi::printLogMessage(const std::string& message, const std::string& message_param, unsigned char severity)
{
	PrintLogMessage(message.c_str(), message_param.c_str(), vehicle_name_.c_str(), severity);
}

std::vector<std::string> WorldSimApi::listSceneObjects(const std::string& name_regex) const
{
	std::vector<std::string> result;
	throw std::invalid_argument(common_utils::Utils::stringf(
		"simListSceneObject is not supported on unity").c_str());
	return result;
}

WorldSimApi::Pose WorldSimApi::getObjectPose(const std::string& object_name) const
{
	AirSimUnity::AirSimPose airSimPose = GetPose(object_name.c_str());
	return UnityUtilities::Convert_to_Pose(airSimPose);
}

bool WorldSimApi::setObjectPose(const std::string& object_name, const WorldSimApi::Pose& pose, bool teleport)
{
	AirSimUnity::AirSimPose airSimPose = UnityUtilities::Convert_to_AirSimPose(pose);
	return SetPose(airSimPose, false, object_name.c_str());
}

void WorldSimApi::enableWeather(bool enable)
{
    unused(enable);
    //TODO: implement weather for Unity
}
void WorldSimApi::setWeatherParameter(WeatherParameter param, float val)
{
    unused(param);
    unused(val);
    //TODO: implement weather for Unity
}

#pragma region Character
// Not implemented, just added for compilation.

void WorldSimApi::charSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name)
{
}

float WorldSimApi::charGetFaceExpression(const std::string& expression_name, const std::string& character_name) const
{
	return 0.0f;
}

std::vector<std::string> WorldSimApi::charGetAvailableFaceExpressions()
{
	return std::vector<std::string>();
}

void WorldSimApi::charSetSkinDarkness(float value, const std::string& character_name)
{
}

float WorldSimApi::charGetSkinDarkness(const std::string& character_name) const
{
	return 0.0f;
}

void WorldSimApi::charSetSkinAgeing(float value, const std::string& character_name)
{
}

float WorldSimApi::charGetSkinAgeing(const std::string& character_name) const
{
	return 0.0f;
}

void WorldSimApi::charSetHeadRotation(const msr::airlib::Quaternionr& q, const std::string& character_name)
{
}

msr::airlib::Quaternionr WorldSimApi::charGetHeadRotation(const std::string& character_name) const
{
	return msr::airlib::Quaternionr();
}

void WorldSimApi::charSetBonePose(const std::string& bone_name, const msr::airlib::Pose& pose, const std::string& character_name)
{
}

msr::airlib::Pose WorldSimApi::charGetBonePose(const std::string& bone_name, const std::string& character_name) const
{
	return Pose();
}

void WorldSimApi::charResetBonePose(const std::string& bone_name, const std::string& character_name)
{
}

void WorldSimApi::charSetFacePreset(const std::string& preset_name, float value, const std::string& character_name)
{
}

void WorldSimApi::charSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name)
{
}

void WorldSimApi::charSetBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses, const std::string& character_name)
{
}

std::unordered_map<std::string, msr::airlib::Pose> WorldSimApi::charGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name) const
{
	return std::unordered_map<std::string, Pose>();
}

#pragma endregion

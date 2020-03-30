
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

#pragma endregion

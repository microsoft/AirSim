#pragma once

#include "api/WorldSimApiBase.hpp"
#include "./SimMode/SimModeBase.h"
#include "AirSimStructs.hpp"

class WorldSimApi : public msr::airlib::WorldSimApiBase
{
public:
	typedef msr::airlib::Pose Pose;

public:
	WorldSimApi(SimModeBase* simmode, std::string vehicle_name);
	virtual ~WorldSimApi();
	virtual bool isPaused() const override;
	virtual void reset() override;
	virtual void pause(bool is_paused) override;
	virtual void continueForTime(double seconds) override;
        virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
            float celestial_clock_speed, float update_interval_secs, bool move_sun);

    virtual void enableWeather(bool enable);
    virtual void setWeatherParameter(WeatherParameter param, float val);

	virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) override;
	virtual int getSegmentationObjectID(const std::string& mesh_name) const override;
	virtual void printLogMessage(const std::string& message,
		const std::string& message_param = "", unsigned char severity = 0) override;

	virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const override;
	virtual Pose getObjectPose(const std::string& object_name) const override;
	virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) override;

private:
	SimModeBase * simmode_;
	std::string vehicle_name_;
};

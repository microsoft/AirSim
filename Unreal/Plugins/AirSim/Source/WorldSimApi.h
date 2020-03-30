#pragma once

#include "CoreMinimal.h"
#include "common/CommonStructs.hpp"
#include "api/WorldSimApiBase.hpp"
#include "SimMode/SimModeBase.h"
#include <string>

class WorldSimApi : public msr::airlib::WorldSimApiBase {
public:
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::Vector3r Vector3r;

    WorldSimApi(ASimModeBase* simmode);
    virtual ~WorldSimApi() = default;

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

	virtual std::unique_ptr<std::vector<std::string>> swapTextures(const std::string& tag, int tex_id = 0, int component_id = 0, int material_id = 0) override;
    virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const override;
    virtual Pose getObjectPose(const std::string& object_name) const override;
    virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) override;

    //----------- Plotting APIs ----------/
    virtual void simFlushPersistentMarkers() override;
    virtual void simPlotPoints(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float size, float duration, bool is_persistent) override;
    virtual void simPlotLineStrip(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent) override;
    virtual void simPlotLineList(const std::vector<Vector3r>& points, const std::vector<float>& color_rgba, float thickness, float duration, bool is_persistent) override;
    virtual void simPlotArrows(const std::vector<Vector3r>& points_start, const std::vector<Vector3r>& points_end, const std::vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent) override;
    virtual void simPlotStrings(const std::vector<std::string>& strings, const std::vector<Vector3r>& positions, float scale, const std::vector<float>& color_rgba, float duration) override;
    virtual void simPlotTransforms(const std::vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent) override;
    virtual void simPlotTransformsWithNames(const std::vector<Pose>& poses, const std::vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const std::vector<float>& text_color_rgba, float duration) override;

private:
    ASimModeBase* simmode_;
};

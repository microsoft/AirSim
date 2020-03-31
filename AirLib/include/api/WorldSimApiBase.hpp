// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WorldSimApiBase_hpp
#define air_WorldSimApiBase_hpp

#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {


class WorldSimApiBase {
public:
    enum class WeatherParameter {
        Rain = 0,
        Roadwetness = 1,
        Snow = 2,
        RoadSnow = 3,
        MapleLeaf = 4,
        RoadLeaf = 5,
        Dust = 6,
        Fog = 7,
        Enabled = 8
    };

    virtual ~WorldSimApiBase() = default;

    virtual bool isPaused() const = 0;
    virtual void reset() = 0;
    virtual void pause(bool is_paused) = 0;
    virtual void continueForTime(double seconds) = 0;

    virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
        float celestial_clock_speed, float update_interval_secs, bool move_sun) = 0;

    virtual void enableWeather(bool enable) = 0;
    virtual void setWeatherParameter(WeatherParameter param, float val) = 0;

    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
    virtual int getSegmentationObjectID(const std::string& mesh_name) const = 0;

    virtual void printLogMessage(const std::string& message,
        const std::string& message_param = "", unsigned char severity = 0) = 0;

    //----------- Plotting APIs ----------/
    virtual void simFlushPersistentMarkers() = 0;
    virtual void simPlotPoints(const vector<Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent) = 0; 
    virtual void simPlotLineStrip(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent) = 0; 
    virtual void simPlotLineList(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent) = 0; 
    virtual void simPlotArrows(const vector<Vector3r>& points_start, const vector<Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent) = 0; 
    virtual void simPlotStrings(const vector<std::string>& strings, const vector<Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration) = 0;
    virtual void simPlotTransforms(const vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent) = 0; 
    virtual void simPlotTransformsWithNames(const vector<Pose>& poses, const vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration) = 0;

    virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const = 0;
    virtual Pose getObjectPose(const std::string& object_name) const = 0;
    virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) = 0;

	virtual std::unique_ptr<std::vector<std::string>> swapTextures(const std::string& tag, int tex_id = 0, int component_id = 0, int material_id = 0) = 0;
};


}} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WorldSimApiBase_hpp
#define air_WorldSimApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/AirSimSettings.hpp"

namespace msr
{
namespace airlib
{

    class WorldSimApiBase
    {
    public:
        enum class WeatherParameter
        {
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

        // ------ Level setting apis ----- //
        virtual bool loadLevel(const std::string& level_name) = 0;
        virtual string spawnObject(string& object_name, const string& load_component, const Pose& pose, const Vector3r& scale, bool physics_enabled) = 0;
        virtual bool destroyObject(const string& object_name) = 0;

        virtual bool isPaused() const = 0;
        virtual void reset() = 0;
        virtual void pause(bool is_paused) = 0;
        virtual void continueForTime(double seconds) = 0;
        virtual void continueForFrames(uint32_t frames) = 0;

        virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
                                  float celestial_clock_speed, float update_interval_secs, bool move_sun) = 0;

        virtual void enableWeather(bool enable) = 0;
        virtual void setWeatherParameter(WeatherParameter param, float val) = 0;

        virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
        virtual int getSegmentationObjectID(const std::string& mesh_name) const = 0;

        virtual bool addVehicle(const std::string& vehicle_name, const std::string& vehicle_type, const Pose& pose, const std::string& pawn_path = "") = 0;

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
        virtual Vector3r getObjectScale(const std::string& object_name) const = 0;
        virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) = 0;
        virtual bool runConsoleCommand(const std::string& command) = 0;
        virtual bool setObjectScale(const std::string& object_name, const Vector3r& scale) = 0;
        virtual std::unique_ptr<std::vector<std::string>> swapTextures(const std::string& tag, int tex_id = 0, int component_id = 0, int material_id = 0) = 0;
        virtual vector<MeshPositionVertexBuffersResponse> getMeshPositionVertexBuffers() const = 0;

        virtual bool createVoxelGrid(const Vector3r& position, const int& x_size, const int& y_size, const int& z_size, const float& res, const std::string& output_file) = 0;

        // Recording APIs
        virtual void startRecording() = 0;
        virtual void stopRecording() = 0;
        virtual bool isRecording() const = 0;

        virtual void setWind(const Vector3r& wind) const = 0;

        virtual std::string getSettingsString() const = 0;
    };
}
} //namespace
#endif

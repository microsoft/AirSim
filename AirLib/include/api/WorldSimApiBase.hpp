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

    virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const = 0;
    virtual Pose getObjectPose(const std::string& object_name) const = 0;
    virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) = 0;

    //----------- APIs to control ACharacter in scene ----------/
    virtual void charSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name) = 0;
    virtual float charGetFaceExpression(const std::string& expression_name, const std::string& character_name) const = 0;
    virtual std::vector<std::string> charGetAvailableFaceExpressions() = 0;
    virtual void charSetSkinDarkness(float value, const std::string& character_name) = 0;
    virtual float charGetSkinDarkness(const std::string& character_name) const = 0;
    virtual void charSetSkinAgeing(float value, const std::string& character_name) = 0;
    virtual float charGetSkinAgeing(const std::string& character_name) const = 0;
    virtual void charSetHeadRotation(const msr::airlib::Quaternionr& q, const std::string& character_name) = 0;
    virtual msr::airlib::Quaternionr charGetHeadRotation(const std::string& character_name) const = 0;
    virtual void charSetBonePose(const std::string& bone_name, const msr::airlib::Pose& pose, const std::string& character_name) = 0;
    virtual msr::airlib::Pose charGetBonePose(const std::string& bone_name, const std::string& character_name) const = 0;
    virtual void charResetBonePose(const std::string& bone_name, const std::string& character_name) = 0;
    virtual void charSetFacePreset(const std::string& preset_name, float value, const std::string& character_name) = 0;
    virtual void charSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name) = 0;
    virtual void charSetBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses, const std::string& character_name) = 0;
    virtual std::unordered_map<std::string, msr::airlib::Pose> charGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name) const = 0;

};


}} //namespace
#endif

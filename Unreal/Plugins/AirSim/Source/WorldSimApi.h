#pragma once

#include "CoreMinimal.h"
#include "common/CommonStructs.hpp"
#include "api/WorldSimApiBase.hpp"
#include "SimMode/SimModeBase.h"
#include "AirSimCharacter.h"
#include <string>

class WorldSimApi : public msr::airlib::WorldSimApiBase {
public:
    typedef msr::airlib::Pose Pose;

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

    virtual std::vector<std::string> listSceneObjects(const std::string& name_regex) const override;
    virtual Pose getObjectPose(const std::string& object_name) const override;
    virtual bool setObjectPose(const std::string& object_name, const Pose& pose, bool teleport) override;

    //----------- APIs to control ACharacter in scene ----------/
    virtual void charSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name) override;
    virtual float charGetFaceExpression(const std::string& expression_name, const std::string& character_name) const override;
    virtual std::vector<std::string> charGetAvailableFaceExpressions() override;
    virtual void charSetSkinDarkness(float value, const std::string& character_name) override;
    virtual float charGetSkinDarkness(const std::string& character_name) const override;
    virtual void charSetSkinAgeing(float value, const std::string& character_name) override;
    virtual float charGetSkinAgeing(const std::string& character_name) const override;
    virtual void charSetHeadRotation(const msr::airlib::Quaternionr& q, const std::string& character_name) override;
    virtual msr::airlib::Quaternionr charGetHeadRotation(const std::string& character_name) const override;
    virtual void charSetBonePose(const std::string& bone_name, const msr::airlib::Pose& pose, const std::string& character_name) override;
    virtual msr::airlib::Pose charGetBonePose(const std::string& bone_name, const std::string& character_name) const override;
    virtual void charResetBonePose(const std::string& bone_name, const std::string& character_name) override;
    virtual void charSetFacePreset(const std::string& preset_name, float value, const std::string& character_name) override;
    virtual void charSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name) override;
    virtual void charSetBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses, const std::string& character_name) override;
    virtual std::unordered_map<std::string, msr::airlib::Pose> charGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name) const override;

private:
    AAirSimCharacter* getAirSimCharacter(const std::string& character_name);
    const AAirSimCharacter* getAirSimCharacter(const std::string& character_name) const;



private:
    ASimModeBase* simmode_;
    std::map<std::string, AAirSimCharacter*> chars_;
};

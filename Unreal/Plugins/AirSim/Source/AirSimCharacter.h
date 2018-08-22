#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Engine/DataTable.h"

#include "common/Common.hpp"
#include "AirSimCharacter.generated.h"

UCLASS()
class AIRSIM_API AAirSimCharacter : public ACharacter
{
    GENERATED_BODY()

public:
    virtual void setFaceExpression(const std::string& expression_name, float value);
    virtual float getFaceExpression(const std::string& expression_name) const;
    virtual std::vector<std::string> getAvailableFaceExpressions() const;
    virtual void setSkinDarkness(float value);
    virtual float getSkinDarkness() const;
    virtual void setSkinAgeing(float value);
    virtual float getSkinAgeing() const;
    virtual void setHeadRotation(const msr::airlib::Quaternionr& q);
    virtual msr::airlib::Quaternionr getHeadRotation() const;
    virtual void setBonePose(const std::string& bone_name, const msr::airlib::Pose& pose);
    virtual void setBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses);
    virtual std::unordered_map<std::string, msr::airlib::Pose> getBonePoses(const std::vector<std::string>& bone_names) const;
    virtual msr::airlib::Pose getBonePose(const std::string& bone_name) const;
    virtual void resetBonePose(const std::string& bone_name);
    virtual void setFacePreset(const std::string& preset_name, float value);
    virtual void setFacePresets(const std::unordered_map<std::string, float>& presets);
    virtual void reset();
};

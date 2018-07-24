#include "WorldSimApi.h"
#include "AirBlueprintLib.h"
#include "common/common_utils/Utils.hpp"

WorldSimApi::WorldSimApi(ASimModeBase* simmode)
    : simmode_(simmode)
{
}

bool WorldSimApi::isPaused() const
{
    return simmode_->isPaused();
}

void WorldSimApi::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        simmode_->reset();

        //reset any chars we have
        for (auto& c : chars_)
            c.second->reset();
    }, true);
}

void WorldSimApi::pause(bool is_paused)
{
    simmode_->pause(is_paused);
}

void WorldSimApi::continueForTime(double seconds)
{
    simmode_->continueForTime(seconds);
}

bool WorldSimApi::setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex)
{
    bool success;
    UAirBlueprintLib::RunCommandOnGameThread([mesh_name, object_id, is_name_regex, &success]() {
        success = UAirBlueprintLib::SetMeshStencilID(mesh_name, object_id, is_name_regex);
    }, true);
    return success;
}

int WorldSimApi::getSegmentationObjectID(const std::string& mesh_name) const
{
    int result;
    UAirBlueprintLib::RunCommandOnGameThread([&mesh_name, &result]() {
        result = UAirBlueprintLib::GetMeshStencilID(mesh_name);
    }, true);
    return result;
}

void WorldSimApi::printLogMessage(const std::string& message,
    const std::string& message_param, unsigned char severity)
{
    UAirBlueprintLib::LogMessageString(message, message_param, static_cast<LogDebugLevel>(severity));
}

WorldSimApi::Pose WorldSimApi::getObjectPose(const std::string& object_name) const
{
    Pose result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &result]() {
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        result = actor ? simmode_->getGlobalNedTransform().toGlobalNed(FTransform(actor->GetActorRotation(), actor->GetActorLocation()))
            : Pose::nanPose();
    }, true);
    return result;
}

bool WorldSimApi::setObjectPose(const std::string& object_name, const WorldSimApi::Pose& pose, bool teleport)
{
    bool result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &object_name, &pose, teleport, &result]() {
        FTransform actor_transform = simmode_->getGlobalNedTransform().fromGlobalNed(pose);
        AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
        if (actor) {
            if (teleport) 
                result = actor->SetActorLocationAndRotation(actor_transform.GetLocation(), actor_transform.GetRotation(), false, nullptr, ETeleportType::TeleportPhysics);
            else
                result = actor->SetActorLocationAndRotation(actor_transform.GetLocation(), actor_transform.GetRotation(), true);
        }
        else
            result = false;
    }, true);
    return result;
}

void WorldSimApi::charSetFaceExpression(const std::string& expression_name, float value, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setFaceExpression(expression_name, value);
}

float WorldSimApi::charGetFaceExpression(const std::string& expression_name, const std::string& character_name) const
{
    const AAirSimCharacter* character = getAirSimCharacter(character_name);
    return character->getFaceExpression(expression_name);
}

std::vector<std::string> WorldSimApi::charGetAvailableFaceExpressions()
{
    const AAirSimCharacter* character = getAirSimCharacter("");
    return character->getAvailableFaceExpressions();
}

void WorldSimApi::charSetSkinDarkness(float value, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setSkinDarkness(value);
}

float WorldSimApi::charGetSkinDarkness(const std::string& character_name) const
{
    const AAirSimCharacter* character = getAirSimCharacter(character_name);
    return character->getSkinDarkness();
}

void WorldSimApi::charSetSkinAgeing(float value, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setSkinAgeing(value);
}

float WorldSimApi::charGetSkinAgeing(const std::string& character_name) const
{
    const AAirSimCharacter* character = getAirSimCharacter(character_name);
    return character->getSkinAgeing();
}

void WorldSimApi::charSetHeadRotation(const msr::airlib::Quaternionr& q, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setHeadRotation(q);
}

msr::airlib::Quaternionr WorldSimApi::charGetHeadRotation(const std::string& character_name) const
{
    const AAirSimCharacter* character = getAirSimCharacter(character_name);
    return character->getHeadRotation();
}

void WorldSimApi::charSetBonePose(const std::string& bone_name, const msr::airlib::Pose& pose, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setBonePose(bone_name, pose);
}

msr::airlib::Pose WorldSimApi::charGetBonePose(const std::string& bone_name, const std::string& character_name) const
{
    const AAirSimCharacter* character = getAirSimCharacter(character_name);
    return character->getBonePose(bone_name);
}

void WorldSimApi::charResetBonePose(const std::string& bone_name, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->resetBonePose(bone_name);
}

void WorldSimApi::charSetFacePreset(const std::string& preset_name, float value, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setFacePreset(preset_name, value);
}

AAirSimCharacter* WorldSimApi::getAirSimCharacter(const std::string& character_name)
{
    AAirSimCharacter* character = nullptr;
    UAirBlueprintLib::RunCommandOnGameThread([this, &character_name, &character]() {
        if (chars_.size() == 0) { //not found in the cache
            TArray<AActor*> characters;
            UAirBlueprintLib::FindAllActor<AAirSimCharacter>(simmode_, characters);
            for (AActor* actor : characters) {
                character = static_cast<AAirSimCharacter*>(actor);
                chars_[std::string(
                    TCHAR_TO_UTF8(*character->GetName()))] = character;
            }
        }

        if (chars_.size() == 0) {
            throw std::invalid_argument(
                "There were no actors of class ACharactor found in the environment");
        }

        //choose first character if name was blank or find by name
        character = character_name == "" ? chars_.begin()->second
            : common_utils::Utils::findOrDefault(chars_, character_name);

        if (!character) {
            throw std::invalid_argument(common_utils::Utils::stringf(
                "Character with name %s was not found in the environment", character_name.c_str()).c_str());
        }
    }, true);

    return character;
}

const AAirSimCharacter* WorldSimApi::getAirSimCharacter(const std::string& character_name) const
{
    return const_cast<WorldSimApi*>(this)->getAirSimCharacter(character_name);
}


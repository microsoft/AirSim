#include "WorldSimApi.h"
#include "AirBlueprintLib.h"
#include "TextureShuffleActor.h"
#include "common/common_utils/Utils.hpp"
#include "Weather/WeatherLib.h"
#include "DrawDebugHelpers.h"

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

void WorldSimApi::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
    float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
    simmode_->setTimeOfDay(is_enabled, start_datetime, is_start_datetime_dst,
        celestial_clock_speed, update_interval_secs, move_sun);
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

std::vector<std::string> WorldSimApi::listSceneObjects(const std::string& name_regex) const
{
    std::vector<std::string> result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &name_regex, &result]() {
        result = UAirBlueprintLib::ListMatchingActors(simmode_, name_regex);
    }, true);
    return result;
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

void WorldSimApi::enableWeather(bool enable)
{
    UWeatherLib::setWeatherEnabled(simmode_->GetWorld(), enable);
}

void WorldSimApi::setWeatherParameter(WeatherParameter param, float val)
{
    unsigned char param_n = static_cast<unsigned char>(msr::airlib::Utils::toNumeric<WeatherParameter>(param));
    EWeatherParamScalar param_e = msr::airlib::Utils::toEnum<EWeatherParamScalar>(param_n);

    UWeatherLib::setWeatherParamScalar(simmode_->GetWorld(), param_e, val);
}

std::unique_ptr<std::vector<std::string>> WorldSimApi::swapTextures(const std::string& tag, int tex_id, int component_id, int material_id)
{
	auto swappedObjectNames = std::make_unique<std::vector<std::string>>();
	UAirBlueprintLib::RunCommandOnGameThread([this, &tag, tex_id, component_id, material_id, &swappedObjectNames]() {
		//Split the tag string into individual tags.
		TArray<FString> splitTags;
		FString notSplit = FString(tag.c_str());
		FString next = "";
		while (notSplit.Split(",", &next, &notSplit))
		{
			next.TrimStartInline();
			splitTags.Add(next);
		}
		notSplit.TrimStartInline();
		splitTags.Add(notSplit);

		//Texture swap on actors that have all of those tags.
		TArray<AActor*> shuffleables;
		UAirBlueprintLib::FindAllActor<ATextureShuffleActor>(simmode_, shuffleables);
		for (auto *shuffler : shuffleables)
		{
			bool invalidChoice = false;
			for (auto required_tag : splitTags)
			{
				invalidChoice |= !shuffler->ActorHasTag(FName(*required_tag));
				if (invalidChoice)
					break;
			}
			
			if (invalidChoice)
				continue;
			dynamic_cast<ATextureShuffleActor*>(shuffler)->SwapTexture(tex_id, component_id, material_id);
			swappedObjectNames->push_back(TCHAR_TO_UTF8(*shuffler->GetName()));
		}
	}, true);
	return swappedObjectNames;
}
//----------- Plotting APIs ----------/
void WorldSimApi::simPlotPoints(const vector<Vector3r>& points, const vector<float>& color_rgba, float size, float duration, bool is_persistent)
{
    FLinearColor color {color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]};
    for (const auto& point : points)
    {
        DrawDebugPoint(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(point), size, color.ToFColor(true), is_persistent, duration);
    }
}

// plot line for points 0-1, 1-2, 2-3
void WorldSimApi::simPlotLineStrip(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    FLinearColor color {color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]};
    for (size_t idx = 0; idx != points.size()-1; idx++)
    {
        DrawDebugLine(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(points[idx]), simmode_->getGlobalNedTransform().fromGlobalNed(points[idx+1]), color.ToFColor(true), is_persistent, duration, 0, thickness);
    }
}

// plot line for points 0-1, 2-3, 4-5... must be even number of points
void WorldSimApi::simPlotLineList(const vector<Vector3r>& points, const vector<float>& color_rgba, float thickness, float duration, bool is_persistent)
{
    if (points.size() % 2)
    {

    }

    FLinearColor color {color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]};
    for (int idx = 0; idx < points.size(); idx += 2)
    {
        DrawDebugLine(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(points[idx]), simmode_->getGlobalNedTransform().fromGlobalNed(points[idx+1]), color.ToFColor(true), is_persistent, duration, 0, thickness);
    }
}

void WorldSimApi::simPlotArrows(const vector<Vector3r>& points_start, const vector<Vector3r>& points_end, const vector<float>& color_rgba, float thickness, float arrow_size, float duration, bool is_persistent)
{
    // assert points_start.size() == poinst_end.size()
    FLinearColor color {color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]};
    for (int idx = 0; idx < points_start.size(); idx += 1)
    {
        DrawDebugDirectionalArrow(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(points_start[idx]), simmode_->getGlobalNedTransform().fromGlobalNed(points_end[idx]), arrow_size, color.ToFColor(true), is_persistent, duration, 0, thickness);
    }
}

void WorldSimApi::simPlotStrings(const vector<std::string>& strings, const vector<Vector3r>& positions, float scale, const vector<float>& color_rgba, float duration)
{
    // assert positions.size() == strings.size()
    FLinearColor color {color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3]};
    for (int idx = 0; idx < positions.size(); idx += 1)
    {
        DrawDebugString(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(positions[idx]), FString(strings[idx].c_str()), NULL, color.ToFColor(true), duration, false, scale);
    }
}

void WorldSimApi::simPlotTransforms(const vector<Pose>& poses, float scale, float thickness, float duration, bool is_persistent)
{
    for (const auto& pose : poses)
    {
        DrawDebugCoordinateSystem(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(pose).GetLocation(), simmode_->getGlobalNedTransform().fromGlobalNed(pose).Rotator(), scale, is_persistent, duration, 0, thickness);
    }
}

void WorldSimApi::simPlotTransformsWithNames(const vector<Pose>& poses, const vector<std::string>& names, float tf_scale, float tf_thickness, float text_scale, const vector<float>& text_color_rgba, float duration)
{
    // assert poses.size() == names.size()
    FLinearColor color {text_color_rgba[0], text_color_rgba[1], text_color_rgba[2], text_color_rgba[3]};
    for (int idx = 0; idx < poses.size(); idx += 1)
    {
        DrawDebugCoordinateSystem(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx]).GetLocation(), simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx]).Rotator(), tf_scale, false, duration, 0, tf_thickness);
        DrawDebugString(simmode_->GetWorld(), simmode_->getGlobalNedTransform().fromGlobalNed(poses[idx]).GetLocation(), FString(names[idx].c_str()), NULL, color.ToFColor(true), duration, false, text_scale);
    }
}

//------------------------------------------------- Char APIs -----------------------------------------------------------/

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

void WorldSimApi::charSetFacePresets(const std::unordered_map<std::string, float>& presets, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setFacePresets(presets);
}
void WorldSimApi::charSetBonePoses(const std::unordered_map<std::string, msr::airlib::Pose>& poses, const std::string& character_name)
{
    AAirSimCharacter* character = getAirSimCharacter(character_name);
    character->setBonePoses(poses);
}
std::unordered_map<std::string, msr::airlib::Pose> WorldSimApi::charGetBonePoses(const std::vector<std::string>& bone_names, const std::string& character_name) const
{
    const AAirSimCharacter* character = getAirSimCharacter(character_name);
    return character->getBonePoses(bone_names);
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
//------------------------------------------------- Char APIs -----------------------------------------------------------/


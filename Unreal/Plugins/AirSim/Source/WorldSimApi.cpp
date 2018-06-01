#include "WorldSimApi.h"
#include "AirBlueprintLib.h"


WorldSimApi::WorldSimApi(ASimModeBase* simmode)
    : simmode_(simmode)
{
    ned_transform_.initialize(simmode_);
}

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

bool WorldSimApi::setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false)
{
    bool success;
    UAirBlueprintLib::RunCommandOnGameThread([mesh_name, object_id, is_name_regex, &success]() {
        success = UAirBlueprintLib::SetMeshStencilID(mesh_name, object_id, is_name_regex);
    }, true);
    return success;
}

int WorldSimApi::getSegmentationObjectID(const std::string& mesh_name) const
{
    return UAirBlueprintLib::GetMeshStencilID(mesh_name);
}

void WorldSimApi::printLogMessage(const std::string& message,
    const std::string& message_param = "", unsigned char severity)
{
    UAirBlueprintLib::LogMessageString(message, message_param, static_cast<LogDebugLevel>(severity));
}

WorldSimApi::Pose WorldSimApi::getObjectPose(const std::string& object_name) const
{
    AActor* actor = UAirBlueprintLib::FindActor<AActor>(simmode_, FString(object_name.c_str()));
    return actor ? toPose(actor->GetActorLocation(), actor->GetActorQuat())
        : Pose::nanPose();
}

WorldSimApi::Pose WorldSimApi::toPose(const FVector& u_position, const FQuat& u_quat) const
{
    const msr::airlib::Vector3r& position = ned_transform_.toNedMeters(u_position);
    const msr::airlib::Quaternionr& orientation = ned_transform_.toQuaternionr(u_quat, true);
    return Pose(position, orientation);
}

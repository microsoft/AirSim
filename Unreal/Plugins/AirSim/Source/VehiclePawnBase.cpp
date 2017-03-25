#include "AirSim.h"
#include "VehiclePawnBase.h"
#include "AirBlueprintLib.h"


void AVehiclePawnBase::PostInitializeComponents()
{
    Super::PostInitializeComponents();
}

void AVehiclePawnBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    state_ = initial_state_ = State();

    Super::EndPlay(EndPlayReason);
}

void AVehiclePawnBase::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    // Deflect along the surface when we collide.
    //FRotator CurrentRotation = GetActorRotation(RootComponent);
    //SetActorRotation(FQuat::Slerp(CurrentRotation.Quaternion(), HitNormal.ToOrientationQuat(), 0.025f));

    state_.collison_info.has_collided = true;
    ++state_.collison_info.collison_count;
    state_.collison_info.normal = AVehiclePawnBase::toVector3r(Hit.ImpactNormal, 1, true);
    state_.collison_info.impact_point = toNedMeters(Hit.ImpactPoint);
    state_.collison_info.position = toNedMeters(getPosition());
    state_.collison_info.penetration_depth = Hit.PenetrationDepth / world_to_meters;

    UAirBlueprintLib::LogMessage(TEXT("Collison Count:"), FString::FromInt(state_.collison_info.collison_count), LogDebugLevel::Failure);
}

void AVehiclePawnBase::initialize()
{
    //set up key variables
    world_to_meters = UAirBlueprintLib::GetWorldToMetersScale(this);
    home_point = msr::airlib::GeoPoint(HomeLatitude, HomeLongitude, HomeAltitude);

    //initialize state
    this->GetActorBounds(true, initial_state_.mesh_origin, initial_state_.mesh_bounds);
    initial_state_.ground_offset = FVector(0, 0, initial_state_.mesh_bounds.Z);
    initial_state_.transformation_offset = this->GetActorLocation() - initial_state_.ground_offset;
    ground_margin = FVector(0, 0, 20); //TODO: can we explain this experimental setting? 7 seems to be minimum
    ground_trace_end = initial_state_.ground_offset + ground_margin; 

    initial_state_.start_location = getPosition();
    initial_state_.last_position = initial_state_.start_location;
    initial_state_.last_debug_position = initial_state_.start_location;
    initial_state_.start_rotation = getOrientation();

    initial_state_.tracing_enabled = EnableTrace;
    initial_state_.collisons_enabled = EnableCollisons;
    initial_state_.passthrough_enabled = EnablePassthroughOnCollisons;


    initial_state_.collison_info = CollisionInfo();

    initial_state_.was_last_move_teleport = false;
    initial_state_.was_last_move_teleport = canTeleportWhileMove();

    reset();
}

APIPCamera* AVehiclePawnBase::getFpvCamera()
{
    //should be overridden in derived class
    return nullptr;
}

void AVehiclePawnBase::reset()
{
    state_ = initial_state_;
    this->SetActorLocation(state_.start_location, false, nullptr, ETeleportType::TeleportPhysics);
    this->SetActorRotation(state_.start_rotation, ETeleportType::TeleportPhysics);
}

const AVehiclePawnBase::GeoPoint& AVehiclePawnBase::getHomePoint() const
{
    return home_point;
}

const AVehiclePawnBase::CollisionInfo& AVehiclePawnBase::getCollisonInfo() const
{
    return state_.collison_info;
}

FVector AVehiclePawnBase::getPosition() const
{
    return this->GetActorLocation(); // - state_.mesh_origin
}

FRotator AVehiclePawnBase::getOrientation() const
{
    return this->GetActorRotation();
}


AVehiclePawnBase::Vector3r AVehiclePawnBase::toNedMeters(const FVector& position) const
{
    return AVehiclePawnBase::toVector3r(position - state_.transformation_offset, 1 / world_to_meters, true);
}
FVector AVehiclePawnBase::toNeuUU(const Vector3r& position) const
{
    return AVehiclePawnBase::toFVector(position, world_to_meters, true) + state_.transformation_offset;
}

void AVehiclePawnBase::toggleTrace()
{
    state_.tracing_enabled = !state_.tracing_enabled;

    if (!state_.tracing_enabled)
        FlushPersistentDebugLines(this->GetWorld());
}

void AVehiclePawnBase::allowPassthroughToggleInput()
{
    state_.passthrough_enabled = !state_.passthrough_enabled;
    UAirBlueprintLib::LogMessage("EnablePassthroughOnCollisons: ", FString::FromInt(state_.passthrough_enabled), LogDebugLevel::Informational);
}


//parameters in NED frame
AVehiclePawnBase::Pose AVehiclePawnBase::getPose() const
{
    Vector3r position = toNedMeters(getPosition());
    Quaternionr orientation = AVehiclePawnBase::toQuaternionr(this->GetActorRotation().Quaternion(), true);
    return Pose(position, orientation);
}

void AVehiclePawnBase::setPose(const Pose& pose, const Pose& debug_pose)
{
    //translate to new AVehiclePawnBase position & orientation from NED to NEU
    FVector position = toNeuUU(pose.position);
    //quaternion formula comes from http://stackoverflow.com/a/40334755/207661
    FQuat orientation = toFQuat(pose.orientation, true);

    bool enable_teleport = canTeleportWhileMove();

    //must reset collison before we set pose. Setting pose will immediately call NotifyHit if there was collison
    //if there was no collison than has_collided would remain false, else it will be set so its value can be
    //checked at the start of next tick
    state_.collison_info.has_collided = false;
    state_.was_last_move_teleport = enable_teleport;

    if (enable_teleport)
        this->SetActorLocationAndRotation(position, orientation, false, nullptr, ETeleportType::TeleportPhysics);
    else
        this->SetActorLocationAndRotation(position, orientation, true);

    if (state_.tracing_enabled && (state_.last_position - position).SizeSquared() > 0.25) {
        DrawDebugLine(this->GetWorld(), state_.last_position, position, FColor::Purple, true, -1.0F, 0, 3.0F);
        state_.last_position = position;
    }

    if (state_.tracing_enabled && !VectorMath::hasNan(debug_pose.position)) {
        FVector debug_position = toNeuUU(debug_pose.position);
        if ((state_.last_debug_position - debug_position).SizeSquared() > 0.25) {
            DrawDebugLine(this->GetWorld(), state_.last_debug_position, debug_position, FColor::Yellow, true, -1.0F, 0, 3.0F);
            state_.last_debug_position = debug_position;
            UAirBlueprintLib::LogMessage("Debug Pose: ", debug_position.ToCompactString(), LogDebugLevel::Informational);
        }
    }
}

bool AVehiclePawnBase::canTeleportWhileMove()  const
{
    //allow teleportation
    //  if collisons are not enabled
    //  or we have collided but passthrough is enabled
    //     we will flip-flop was_last_move_teleport flag so on one tick we have passthrough and other tick we don't
    //     without flip flopping, collisons can't be detected
    return !state_.collisons_enabled || (state_.collison_info.has_collided && !state_.was_last_move_teleport && state_.passthrough_enabled);
}

AVehiclePawnBase::real_T AVehiclePawnBase::getMinZOverGround() const
{
    FVector location = this->GetActorLocation();
    if (UAirBlueprintLib::HasObstacle(this, location, location - ground_trace_end))
        return toNedMeters(location).z();
    else
        return Utils::max<float>();
}

FVector AVehiclePawnBase::toFVector(const Vector3r& vec, float scale, bool convert_from_ned)
{
    return FVector(vec.x() * scale, vec.y() * scale, 
        (convert_from_ned ? -vec.z() : vec.z()) * scale);
}

FQuat AVehiclePawnBase::toFQuat(const Quaternionr& q, bool convert_from_ned)
{
    return FQuat(
        convert_from_ned ? -q.x() : q.x(), 
        convert_from_ned ? -q.y() : q.y(), 
        q.z(), q.w());
}


AVehiclePawnBase::Vector3r AVehiclePawnBase::toVector3r(const FVector& vec, float scale, bool convert_to_ned)
{
    return Vector3r(vec.X * scale, vec.Y * scale, 
        (convert_to_ned ? -vec.Z : vec.Z)  * scale);
}

AVehiclePawnBase::Quaternionr AVehiclePawnBase::toQuaternionr(const FQuat& q, bool convert_to_ned)
{
    return Quaternionr(q.W, 
        convert_to_ned ? -q.X : q.X, 
        convert_to_ned ? -q.Y : q.Y, 
        q.Z);
}
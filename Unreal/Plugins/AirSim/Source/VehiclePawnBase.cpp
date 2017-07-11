#include "AirSim.h"
#include "VehiclePawnBase.h"
#include "AirBlueprintLib.h"
#include "common/ClockFactory.hpp"
#include "NedTransform.h"


AVehiclePawnBase::AVehiclePawnBase()
{
    static ConstructorHelpers::FObjectFinder<UParticleSystem> collison_display(TEXT("ParticleSystem'/AirSim/StarterContent/Particles/P_Explosion.P_Explosion'"));
    if (!collison_display.Succeeded())
        collison_display_template = collison_display.Object;
    else
        collison_display_template = nullptr;
}

void AVehiclePawnBase::PostInitializeComponents()
{
    Super::PostInitializeComponents();
}

void AVehiclePawnBase::BeginPlay()
{
    Super::BeginPlay();
}

void AVehiclePawnBase::setupCamerasFromSettings()
{
    typedef msr::airlib::Settings Settings;
    typedef msr::airlib::VehicleCameraBase::ImageType_ ImageType_;

    Settings& settings = Settings::singleton();
    Settings scene_settings_child, depth_settings_child, seg_settings_child;
    APIPCamera::CaptureSettings scene_settings, depth_settings, seg_settings;
    if (settings.getChild("SceneCaptureSettings", scene_settings_child))
        createCaptureSettings(scene_settings_child, scene_settings);
    if (settings.getChild("DepthCaptureSettings", depth_settings_child))
        createCaptureSettings(depth_settings_child, depth_settings);
    if (settings.getChild("SegCaptureSettings", seg_settings_child))
        createCaptureSettings(seg_settings_child, seg_settings);


    for (int camera_index = 0; camera_index < getCameraCount(); ++camera_index) {
        APIPCamera* camera = getCamera(camera_index);
        camera->setCaptureSettings(ImageType_::Scene, scene_settings);
        camera->setCaptureSettings(ImageType_::Depth, depth_settings);
        camera->setCaptureSettings(ImageType_::Segmentation, seg_settings);
    }
}

void AVehiclePawnBase::createCaptureSettings(const msr::airlib::Settings& settings, APIPCamera::CaptureSettings& capture_settings)
{
    typedef msr::airlib::Settings Settings;

    capture_settings.width = settings.getInt("Width", capture_settings.width);
    capture_settings.height = settings.getInt("Height", capture_settings.height);
    capture_settings.fov_degrees = settings.getFloat("FOV_Degrees", capture_settings.fov_degrees);
    capture_settings.auto_exposure_speed = settings.getFloat("AutoExposureSpeed", capture_settings.auto_exposure_speed);
    capture_settings.motion_blur_amount = settings.getFloat("MotionBlurAmount", capture_settings.motion_blur_amount);
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
    state_.collison_info.normal = NedTransform::toVector3r(Hit.ImpactNormal, 1, true);
    state_.collison_info.impact_point = NedTransform::toNedMeters(Hit.ImpactPoint);
    state_.collison_info.position = NedTransform::toNedMeters(getPosition());
    state_.collison_info.penetration_depth = NedTransform::toNedMeters(Hit.PenetrationDepth);
    state_.collison_info.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
}

void AVehiclePawnBase::displayCollisonEffect(FVector hit_location, const FHitResult& hit)
{
    if (collison_display_template != nullptr && Utils::isDefinitelyLessThan(hit.ImpactNormal.Z, 0.0f)) {
        UParticleSystemComponent* particles = UGameplayStatics::SpawnEmitterAtLocation(this->GetWorld(), collison_display_template, hit_location);
        particles->SetWorldScale3D(FVector(0.1f, 0.1f, 0.1f));
    }
}

void AVehiclePawnBase::initialize()
{
    if (!NedTransform::isInitialized())
        NedTransform::initialize(this);

    //set up key variables
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

void AVehiclePawnBase::initializeForBeginPlay()
{
    //nothing to do in base
}


APIPCamera* AVehiclePawnBase::getCamera(int index)
{
    //should be overridden in derived class
    return nullptr;
}

int AVehiclePawnBase::getCameraCount()
{
    return 0;
}

void AVehiclePawnBase::reset()
{
    state_ = initial_state_;
    this->SetActorLocation(state_.start_location, false, nullptr, ETeleportType::TeleportPhysics);
    this->SetActorRotation(state_.start_rotation, ETeleportType::TeleportPhysics);

    //TODO: delete below
    //std::ifstream sim_log("C:\\temp\\mavlogs\\circle\\sim_cmd_006_orbit 5 1.txt.pos.txt");
    //plot(sim_log, FColor::Purple, Vector3r(0, 0, -3));
    //std::ifstream real_log("C:\\temp\\mavlogs\\circle\\real_cmd_006_orbit 5 1.txt.pos.txt");
    //plot(real_log, FColor::Yellow, Vector3r(0, 0, -3));

    //std::ifstream sim_log("C:\\temp\\mavlogs\\square\\sim_cmd_005_square 5 1.txt.pos.txt");
    //plot(sim_log, FColor::Purple, Vector3r(0, 0, -3));
    //std::ifstream real_log("C:\\temp\\mavlogs\\square\\real_cmd_012_square 5 1.txt.pos.txt");
    //plot(real_log, FColor::Yellow, Vector3r(0, 0, -3));
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

void AVehiclePawnBase::toggleTrace()
{
    state_.tracing_enabled = !state_.tracing_enabled;

    if (!state_.tracing_enabled)
        FlushPersistentDebugLines(this->GetWorld());
    else {     
        state_.debug_position_offset = state_.current_debug_position - state_.current_position;
        state_.last_debug_position = state_.last_position;
    }
}

void AVehiclePawnBase::allowPassthroughToggleInput()
{
    state_.passthrough_enabled = !state_.passthrough_enabled;
    UAirBlueprintLib::LogMessage("EnablePassthroughOnCollisons: ", FString::FromInt(state_.passthrough_enabled), LogDebugLevel::Informational);
}


void AVehiclePawnBase::plot(std::istream& s, FColor color, const Vector3r& offset)
{
    using namespace msr::airlib;

    Vector3r last_point = VectorMath::nanVector();
    uint64_t timestamp;
    float heading, x, y, z;
    while (s >> timestamp >> heading >> x >> y >> z) {
        std::string discarded_line;
        std::getline(s, discarded_line);

        Vector3r current_point(x, y, z);
        current_point += offset;
        if (!VectorMath::hasNan(last_point)) {
            DrawDebugLine(this->GetWorld(), NedTransform::toNeuUU(last_point), NedTransform::toNeuUU(current_point), color, true, -1.0F, 0, 3.0F);
        }
        last_point = current_point;
    }

}

//parameters in NED frame
AVehiclePawnBase::Pose AVehiclePawnBase::getPose() const
{
    Vector3r position = NedTransform::toNedMeters(getPosition());
    Quaternionr orientation = NedTransform::toQuaternionr(this->GetActorRotation().Quaternion(), true);
    return Pose(position, orientation);
}

void AVehiclePawnBase::setPose(const Pose& pose, const Pose& debug_pose)
{
    //translate to new AVehiclePawnBase position & orientation from NED to NEU
    FVector position = NedTransform::toNeuUU(pose.position);
    state_.current_position = position;

    //quaternion formula comes from http://stackoverflow.com/a/40334755/207661
    FQuat orientation = NedTransform::toFQuat(pose.orientation, true);

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
        DrawDebugLine(this->GetWorld(), state_.last_position, position, FColor::Purple, true, -1.0F, 0, 10.0F);
        state_.last_position = position;
    }
    else if (!state_.tracing_enabled) {
        state_.last_position = position;
    }
    state_.current_debug_position = NedTransform::toNeuUU(debug_pose.position);
    if (state_.tracing_enabled && !VectorMath::hasNan(debug_pose.position)) {
        FVector debug_position = state_.current_debug_position - state_.debug_position_offset;
        if ((state_.last_debug_position - debug_position).SizeSquared() > 0.25) {
            DrawDebugLine(this->GetWorld(), state_.last_debug_position, debug_position, FColor(0xaa, 0x33, 0x11), true, -1.0F, 0, 10.0F);
            UAirBlueprintLib::LogMessage("Debug Pose: ", debug_position.ToCompactString(), LogDebugLevel::Informational);
            state_.last_debug_position = debug_position;
        }
    }
    else if (!state_.tracing_enabled) {
        state_.last_debug_position = state_.current_debug_position - state_.debug_position_offset;
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
        return NedTransform::toNedMeters(location).z();
    else
        return Utils::max<float>();
}


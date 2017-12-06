#include "VehiclePawnWrapper.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Particles/ParticleSystemComponent.h"
#include "ConstructorHelpers.h"
#include "AirBlueprintLib.h"
#include "common/ClockFactory.hpp"
#include "NedTransform.h"


VehiclePawnWrapper::VehiclePawnWrapper()
{
    static ConstructorHelpers::FObjectFinder<UParticleSystem> collision_display(TEXT("ParticleSystem'/AirSim/StarterContent/Particles/P_Explosion.P_Explosion'"));
    if (!collision_display.Succeeded())
        collision_display_template = collision_display.Object;
    else
        collision_display_template = nullptr;
}

void VehiclePawnWrapper::setupCamerasFromSettings()
{
    typedef msr::airlib::Settings Settings;
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    Settings& json_settings_root = Settings::singleton();
    Settings json_settings_parent;
    if (json_settings_root.getChild("CaptureSettings", json_settings_parent)) {
        for (size_t child_index = 0; child_index < json_settings_parent.size(); ++child_index) {
            Settings json_settings_child;     
            if (json_settings_parent.getChild(child_index, json_settings_child)) {
                APIPCamera::CaptureSettings capture_settings;;
                createCaptureSettings(json_settings_child, capture_settings);

                int image_type = json_settings_child.getInt("ImageType", -1);

                if (image_type == -1) {
                    UAirBlueprintLib::LogMessageString("ImageType not set in <CaptureSettings> element(s) in settings.json", 
                        std::to_string(child_index), LogDebugLevel::Failure);
                    continue;
                }

                for (int camera_index = 0; camera_index < getCameraCount(); ++camera_index) {
                    APIPCamera* camera = getCamera(camera_index);
                    camera->setCaptureSettings(Utils::toEnum<ImageType>(image_type), capture_settings);
                }
            }
        }
    }
}

void VehiclePawnWrapper::createCaptureSettings(const msr::airlib::Settings& settings, APIPCamera::CaptureSettings& capture_settings)
{
    typedef msr::airlib::Settings Settings;

    capture_settings.width = settings.getInt("Width", capture_settings.width);
    capture_settings.height = settings.getInt("Height", capture_settings.height);
    capture_settings.fov_degrees = settings.getFloat("FOV_Degrees", capture_settings.fov_degrees);
    capture_settings.auto_exposure_speed = settings.getFloat("AutoExposureSpeed", capture_settings.auto_exposure_speed);
    capture_settings.auto_exposure_bias = settings.getFloat("AutoExposureBias", capture_settings.auto_exposure_bias);
    capture_settings.auto_exposure_max_brightness = settings.getFloat("AutoExposureMaxBrightness", capture_settings.auto_exposure_max_brightness);
    capture_settings.auto_exposure_min_brightness = settings.getFloat("AutoExposureMinBrightness", capture_settings.auto_exposure_min_brightness);
    capture_settings.motion_blur_amount = settings.getFloat("MotionBlurAmount", capture_settings.motion_blur_amount);
    capture_settings.target_gamma = settings.getFloat("TargetGamma", capture_settings.target_gamma);
}


void VehiclePawnWrapper::onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    // Deflect along the surface when we collide.
    //FRotator CurrentRotation = GetActorRotation(RootComponent);
    //SetActorRotation(FQuat::Slerp(CurrentRotation.Quaternion(), HitNormal.ToOrientationQuat(), 0.025f));

    UPrimitiveComponent* comp = Cast<class UPrimitiveComponent>(Other ? (Other->GetRootComponent() ? Other->GetRootComponent() : nullptr) : nullptr);

    state_.collision_info.has_collided = true;
    state_.collision_info.normal = NedTransform::toVector3r(Hit.ImpactNormal, 1, true);
    state_.collision_info.impact_point = NedTransform::toNedMeters(Hit.ImpactPoint);
    state_.collision_info.position = NedTransform::toNedMeters(getPosition());
    state_.collision_info.penetration_depth = NedTransform::toNedMeters(Hit.PenetrationDepth);
    state_.collision_info.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
    state_.collision_info.object_name = std::string(Other ? TCHAR_TO_UTF8(*(Other->GetName())) : "(null)");
    state_.collision_info.object_id = comp ? comp->CustomDepthStencilValue : -1;

    ++state_.collision_info.collision_count;


    UAirBlueprintLib::LogMessageString("Collision", Utils::stringf("#%d with %s - ObjID %d", 
        state_.collision_info.collision_count, 
        state_.collision_info.object_name.c_str(), state_.collision_info.object_id),
        LogDebugLevel::Failure);
}

APawn* VehiclePawnWrapper::getPawn()
{
    return pawn_;
}

void VehiclePawnWrapper::displayCollisionEffect(FVector hit_location, const FHitResult& hit)
{
    if (collision_display_template != nullptr && Utils::isDefinitelyLessThan(hit.ImpactNormal.Z, 0.0f)) {
        UParticleSystemComponent* particles = UGameplayStatics::SpawnEmitterAtLocation(pawn_->GetWorld(), collision_display_template, hit_location);
        particles->SetWorldScale3D(FVector(0.1f, 0.1f, 0.1f));
    }
}

const msr::airlib::Kinematics::State* VehiclePawnWrapper::getKinematics()
{
    return kinematics_;
}

void VehiclePawnWrapper::setKinematics(const msr::airlib::Kinematics::State* kinematics)
{
    kinematics_ = kinematics;
}

void VehiclePawnWrapper::initialize(APawn* pawn, const std::vector<APIPCamera*>& cameras, const WrapperConfig& config)
{
    pawn_ = pawn;
    cameras_ = cameras;
    config_ = config;

    for (auto camera : cameras_) {
        camera_connectors_.push_back(std::unique_ptr<VehicleCameraConnector>(new VehicleCameraConnector(camera)));
    }

    if (!NedTransform::isInitialized())
        NedTransform::initialize(pawn_);

    //set up key variables
    home_point_ = msr::airlib::GeoPoint(config.home_lattitude, config.home_longitude, config.home_altitude);

    //initialize state
    pawn_->GetActorBounds(true, initial_state_.mesh_origin, initial_state_.mesh_bounds);
    initial_state_.ground_offset = FVector(0, 0, initial_state_.mesh_bounds.Z);
    initial_state_.transformation_offset = pawn_->GetActorLocation() - initial_state_.ground_offset;
    ground_margin_ = FVector(0, 0, 20); //TODO: can we explain pawn_ experimental setting? 7 seems to be minimum
    ground_trace_end_ = initial_state_.ground_offset + ground_margin_; 

    initial_state_.start_location = getPosition();
    initial_state_.last_position = initial_state_.start_location;
    initial_state_.last_debug_position = initial_state_.start_location;
    initial_state_.start_rotation = getOrientation();

    initial_state_.tracing_enabled = config.enable_trace;
    initial_state_.collisions_enabled = config.enable_collisions;
    initial_state_.passthrough_enabled = config.enable_passthrough_on_collisions;

    initial_state_.collision_info = CollisionInfo();

    initial_state_.was_last_move_teleport = false;
    initial_state_.was_last_move_teleport = canTeleportWhileMove();

    setupCamerasFromSettings();
}

VehiclePawnWrapper::WrapperConfig& VehiclePawnWrapper::getConfig()
{
    return config_;
}

const VehiclePawnWrapper::WrapperConfig& VehiclePawnWrapper::getConfig() const
{
    return config_;
}

APIPCamera* VehiclePawnWrapper::getCamera(int index)
{
    if (index < 0 || index >= cameras_.size())
        throw std::out_of_range("Camera id is not valid");
    //should be overridden in derived class
    return cameras_.at(index);
}

VehicleCameraConnector* VehiclePawnWrapper::getCameraConnector(int index)
{
    //should be overridden in derived class
    return camera_connectors_.at(index).get();
}

int VehiclePawnWrapper::getCameraCount()
{
    return cameras_.size();
}

void VehiclePawnWrapper::reset()
{
    state_ = initial_state_;
    //if (pawn_->GetRootPrimitiveComponent()->IsAnySimulatingPhysics()) {
    //    pawn_->GetRootPrimitiveComponent()->SetSimulatePhysics(false);
    //    pawn_->GetRootPrimitiveComponent()->SetSimulatePhysics(true);
    //}
    pawn_->SetActorLocationAndRotation(state_.start_location, state_.start_rotation, false, nullptr, ETeleportType::TeleportPhysics);

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

const VehiclePawnWrapper::GeoPoint& VehiclePawnWrapper::getHomePoint() const
{
    return home_point_;
}

const VehiclePawnWrapper::CollisionInfo& VehiclePawnWrapper::getCollisionInfo() const
{
    return state_.collision_info;
}

FVector VehiclePawnWrapper::getPosition() const
{
    return pawn_->GetActorLocation(); // - state_.mesh_origin
}

FRotator VehiclePawnWrapper::getOrientation() const
{
    return pawn_->GetActorRotation();
}

void VehiclePawnWrapper::toggleTrace()
{
    state_.tracing_enabled = !state_.tracing_enabled;

    if (!state_.tracing_enabled)
        UKismetSystemLibrary::FlushPersistentDebugLines(pawn_->GetWorld());
    else {     
        state_.debug_position_offset = state_.current_debug_position - state_.current_position;
        state_.last_debug_position = state_.last_position;
    }
}

void VehiclePawnWrapper::allowPassthroughToggleInput()
{
    state_.passthrough_enabled = !state_.passthrough_enabled;
    UAirBlueprintLib::LogMessage("enable_passthrough_on_collisions: ", FString::FromInt(state_.passthrough_enabled), LogDebugLevel::Informational);
}


void VehiclePawnWrapper::plot(std::istream& s, FColor color, const Vector3r& offset)
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
            UKismetSystemLibrary::DrawDebugLine(pawn_->GetWorld(), NedTransform::toNeuUU(last_point), NedTransform::toNeuUU(current_point), color, 0, 3.0F);
        }
        last_point = current_point;
    }

}

void VehiclePawnWrapper::printLogMessage(const std::string& message, std::string message_param, unsigned char severity)
{
    UAirBlueprintLib::LogMessageString(message, message_param, static_cast<LogDebugLevel>(severity));
}

//parameters in NED frame
VehiclePawnWrapper::Pose VehiclePawnWrapper::getPose() const
{
    const Vector3r& position = NedTransform::toNedMeters(getPosition());
    const Quaternionr& orientation = NedTransform::toQuaternionr(pawn_->GetActorRotation().Quaternion(), true);
    return Pose(position, orientation);
}

void VehiclePawnWrapper::setPose(const Pose& pose, bool ignore_collision)
{
    //translate to new VehiclePawnWrapper position & orientation from NED to NEU
    FVector position = NedTransform::toNeuUU(pose.position);
    state_.current_position = position;

    //quaternion formula comes from http://stackoverflow.com/a/40334755/207661
    FQuat orientation = NedTransform::toFQuat(pose.orientation, true);

    bool enable_teleport = ignore_collision || canTeleportWhileMove();

    //must reset collision before we set pose. Setting pose will immediately call NotifyHit if there was collision
    //if there was no collision than has_collided would remain false, else it will be set so its value can be
    //checked at the start of next tick
    state_.collision_info.has_collided = false;
    state_.was_last_move_teleport = enable_teleport;

    if (enable_teleport)
        pawn_->SetActorLocationAndRotation(position, orientation, false, nullptr, ETeleportType::TeleportPhysics);
    else
        pawn_->SetActorLocationAndRotation(position, orientation, true);

    if (state_.tracing_enabled && (state_.last_position - position).SizeSquared() > 0.25) {
        UKismetSystemLibrary::DrawDebugLine(pawn_->GetWorld(), state_.last_position, position, FColor::Purple, -1, 3.0f);
        state_.last_position = position;
    }
    else if (!state_.tracing_enabled) {
        state_.last_position = position;
    }

}

void VehiclePawnWrapper::setDebugPose(const Pose& debug_pose)
{
    state_.current_debug_position = NedTransform::toNeuUU(debug_pose.position);
    if (state_.tracing_enabled && !VectorMath::hasNan(debug_pose.position)) {
        FVector debug_position = state_.current_debug_position - state_.debug_position_offset;
        if ((state_.last_debug_position - debug_position).SizeSquared() > 0.25) {
            UKismetSystemLibrary::DrawDebugLine(pawn_->GetWorld(), state_.last_debug_position, debug_position, FColor(0xaa, 0x33, 0x11), -1, 10.0F);
            UAirBlueprintLib::LogMessage("Debug Pose: ", debug_position.ToCompactString(), LogDebugLevel::Informational);
            state_.last_debug_position = debug_position;
        }
    }
    else if (!state_.tracing_enabled) {
        state_.last_debug_position = state_.current_debug_position - state_.debug_position_offset;
    }
}

bool VehiclePawnWrapper::canTeleportWhileMove()  const
{
    //allow teleportation
    //  if collisions are not enabled
    //  or we have collided but passthrough is enabled
    //     we will flip-flop was_last_move_teleport flag so on one tick we have passthrough and other tick we don't
    //     without flip flopping, collisions can't be detected
    return !state_.collisions_enabled || (state_.collision_info.has_collided && !state_.was_last_move_teleport && state_.passthrough_enabled);
}

void VehiclePawnWrapper::setLogLine(std::string line)
{
    log_line_ = line;
}

std::string VehiclePawnWrapper::getLogLine()
{
    return log_line_;
}




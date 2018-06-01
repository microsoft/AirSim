#include "VehicleSimApi.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Particles/ParticleSystemComponent.h"
#include "ConstructorHelpers.h"
#include "AirBlueprintLib.h"
#include "common/ClockFactory.hpp"
#include "NedTransform.h"
#include "common/EarthUtils.hpp"

VehicleSimApi::VehicleSimApi()
{
    static ConstructorHelpers::FObjectFinder<UParticleSystem> collision_display(TEXT("ParticleSystem'/AirSim/StarterContent/Particles/P_Explosion.P_Explosion'"));
    if (!collision_display.Succeeded())
        collision_display_template = collision_display.Object;
    else
        collision_display_template = nullptr;
}

void VehicleSimApi::setupCamerasFromSettings()
{
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;

    int image_count = static_cast<int>(Utils::toNumeric(ImageType::Count));
    for (int image_type = -1; image_type < image_count; ++image_type) {
        for (int camera_index = 0; camera_index < getCameraCount(); ++camera_index) {
            APIPCamera* camera = getCamera(camera_index);
            camera->setImageTypeSettings(image_type, AirSimSettings::singleton().capture_settings[image_type], 
                AirSimSettings::singleton().noise_settings[image_type]);
        }
    }
}

void VehicleSimApi::onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    // Deflect along the surface when we collide.
    //FRotator CurrentRotation = GetActorRotation(RootComponent);
    //SetActorRotation(FQuat::Slerp(CurrentRotation.Quaternion(), HitNormal.ToOrientationQuat(), 0.025f));

    UPrimitiveComponent* comp = Cast<class UPrimitiveComponent>(Other ? (Other->GetRootComponent() ? Other->GetRootComponent() : nullptr) : nullptr);

    state_.collision_info.has_collided = true;
    state_.collision_info.normal = Vector3r(Hit.ImpactNormal.X, Hit.ImpactNormal.Y, - Hit.ImpactNormal.Z);
    state_.collision_info.impact_point = ned_transform_.toNedMeters(Hit.ImpactPoint);
    state_.collision_info.position = ned_transform_.toNedMeters(getUUPosition());
    state_.collision_info.penetration_depth = ned_transform_.toNedMeters(Hit.PenetrationDepth);
    state_.collision_info.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
    state_.collision_info.object_name = std::string(Other ? TCHAR_TO_UTF8(*(Other->GetName())) : "(null)");
    state_.collision_info.object_id = comp ? comp->CustomDepthStencilValue : -1;

    ++state_.collision_info.collision_count;


    UAirBlueprintLib::LogMessageString("Collision", Utils::stringf("#%d with %s - ObjID %d", 
        state_.collision_info.collision_count, 
        state_.collision_info.object_name.c_str(), state_.collision_info.object_id),
        LogDebugLevel::Failure);
}

void VehicleSimApi::possess()
{
    APlayerController* controller = pawn_->GetWorld()->GetFirstPlayerController();
    controller->UnPossess();
    controller->Possess(pawn_);
}

const NedTransform& VehicleSimApi::getNedTransform() const
{
    return ned_transform_;
}

APawn* VehicleSimApi::getPawn()
{
    return pawn_;
}

std::vector<VehicleSimApi::ImageCaptureBase::ImageResponse> VehicleSimApi::getImages(
    const std::vector<ImageCaptureBase::ImageRequest>& requests) const
{
    std::vector<ImageCaptureBase::ImageResponse> responses;

    const ImageCaptureBase* camera = getImageCapture();
    camera->getImages(requests, responses);

    return responses;
}

std::vector<uint8_t> VehicleSimApi::getImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) const
{
    std::vector<ImageCaptureBase::ImageRequest> request = { ImageCaptureBase::ImageRequest(camera_id, image_type) };
    const std::vector<ImageCaptureBase::ImageResponse>& response = getImages(request);
    if (response.size() > 0)
        return response.at(0).image_data_uint8;
    else
        return std::vector<uint8_t>();
}

void VehicleSimApi::displayCollisionEffect(FVector hit_location, const FHitResult& hit)
{
    if (collision_display_template != nullptr && Utils::isDefinitelyLessThan(hit.ImpactNormal.Z, 0.0f)) {
        UParticleSystemComponent* particles = UGameplayStatics::SpawnEmitterAtLocation(pawn_->GetWorld(), 
            collision_display_template, FTransform(hit_location), true);
        particles->SetWorldScale3D(FVector(0.1f, 0.1f, 0.1f));
    }
}

const msr::airlib::Kinematics::State* VehicleSimApi::getGroundTruthKinematics() const
{
    return kinematics_;
}

void VehicleSimApi::setGroundTruthKinematics(const msr::airlib::Kinematics::State* kinematics)
{
    kinematics_ = kinematics;
}

int VehicleSimApi::getRemoteControlID() const
{
    return vehicle_setting_->rc.remote_control_id;
}

void VehicleSimApi::initialize(APawn* pawn, const std::vector<APIPCamera*>& cameras, const std::string& vehicle_name, 
    const WrapperConfig& config)
{
    typedef msr::airlib::AirSimSettings AirSimSettings;

    pawn_ = pawn;
    cameras_ = cameras;
    config_ = config;
    vehicle_name_ = vehicle_name;
    vehicle_setting_ = AirSimSettings::singleton().getVehicleSetting(vehicle_name_);

    image_capture_.reset(new UnrealImageCapture(cameras_));

    ned_transform_.initialize(pawn);

    //initialize state
    pawn_->GetActorBounds(true, initial_state_.mesh_origin, initial_state_.mesh_bounds);
    initial_state_.ground_offset = FVector(0, 0, initial_state_.mesh_bounds.Z);
    initial_state_.transformation_offset = pawn_->GetActorLocation() - initial_state_.ground_offset;
    ground_margin_ = FVector(0, 0, 20); //TODO: can we explain pawn_ experimental setting? 7 seems to be minimum
    ground_trace_end_ = initial_state_.ground_offset + ground_margin_; 

    initial_state_.start_location = getUUPosition();
    initial_state_.last_position = initial_state_.start_location;
    initial_state_.last_debug_position = initial_state_.start_location;
    initial_state_.start_rotation = getUUOrientation();

    //compute our home point
    Vector3r nedWrtOrigin = ned_transform_.toNedMeters(getUUPosition(), false);
    home_geo_point_ = msr::airlib::EarthUtils::nedToGeodetic(nedWrtOrigin, AirSimSettings::singleton().origin_geopoint);

    initial_state_.tracing_enabled = config.enable_trace;
    initial_state_.collisions_enabled = config.enable_collisions;
    initial_state_.passthrough_enabled = config.enable_passthrough_on_collisions;

    initial_state_.collision_info = CollisionInfo();

    initial_state_.was_last_move_teleport = false;
    initial_state_.was_last_move_teleport = canTeleportWhileMove();

    setupCamerasFromSettings();
}

VehicleSimApi::WrapperConfig& VehicleSimApi::getConfig()
{
    return config_;
}

const VehicleSimApi::VehicleSetting* VehicleSimApi::getVehicleSetting() const
{
    return vehicle_setting_;
}

const VehicleSimApi::WrapperConfig& VehicleSimApi::getConfig() const
{
    return config_;
}

const APIPCamera* VehicleSimApi::getCamera(int index) const
{
    if (index < 0 || index >= cameras_.size())
        throw std::out_of_range("Camera id is not valid");
    //should be overridden in derived class
    return cameras_.at(index);
}

APIPCamera* VehicleSimApi::getCamera(int index)
{
    return const_cast<APIPCamera*>(
        static_cast<const VehicleSimApi*>(this)->getCamera(index));
}

const UnrealImageCapture* VehicleSimApi::getImageCapture() const
{
    return image_capture_.get();
}

int VehicleSimApi::getCameraCount()
{
    return cameras_.size();
}

void VehicleSimApi::reset()
{
    state_ = initial_state_;
    pawn_->SetActorLocationAndRotation(state_.start_location, state_.start_rotation, false, nullptr, ETeleportType::TeleportPhysics);
}

//void playBack()
//{
    //if (pawn_->GetRootPrimitiveComponent()->IsAnySimulatingPhysics()) {
    //    pawn_->GetRootPrimitiveComponent()->SetSimulatePhysics(false);
    //    pawn_->GetRootPrimitiveComponent()->SetSimulatePhysics(true);
    //}
    //TODO: refactor below code used for playback
    //std::ifstream sim_log("C:\\temp\\mavlogs\\circle\\sim_cmd_006_orbit 5 1.txt.pos.txt");
    //plot(sim_log, FColor::Purple, Vector3r(0, 0, -3));
    //std::ifstream real_log("C:\\temp\\mavlogs\\circle\\real_cmd_006_orbit 5 1.txt.pos.txt");
    //plot(real_log, FColor::Yellow, Vector3r(0, 0, -3));

    //std::ifstream sim_log("C:\\temp\\mavlogs\\square\\sim_cmd_005_square 5 1.txt.pos.txt");
    //plot(sim_log, FColor::Purple, Vector3r(0, 0, -3));
    //std::ifstream real_log("C:\\temp\\mavlogs\\square\\real_cmd_012_square 5 1.txt.pos.txt");
    //plot(real_log, FColor::Yellow, Vector3r(0, 0, -3));
//}


VehicleSimApi::CollisionInfo VehicleSimApi::getCollisionInfo() const
{
    return state_.collision_info;
}

FVector VehicleSimApi::getUUPosition() const
{
    return pawn_->GetActorLocation(); // - state_.mesh_origin
}

FRotator VehicleSimApi::getUUOrientation() const
{
    return pawn_->GetActorRotation();
}

void VehicleSimApi::toggleTrace()
{
    state_.tracing_enabled = !state_.tracing_enabled;

    if (!state_.tracing_enabled)
        UKismetSystemLibrary::FlushPersistentDebugLines(pawn_->GetWorld());
    else {     
        state_.debug_position_offset = state_.current_debug_position - state_.current_position;
        state_.last_debug_position = state_.last_position;
    }
}

void VehicleSimApi::allowPassthroughToggleInput()
{
    state_.passthrough_enabled = !state_.passthrough_enabled;
    UAirBlueprintLib::LogMessage("enable_passthrough_on_collisions: ", FString::FromInt(state_.passthrough_enabled), LogDebugLevel::Informational);
}


void VehicleSimApi::plot(std::istream& s, FColor color, const Vector3r& offset)
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
            UKismetSystemLibrary::DrawDebugLine(pawn_->GetWorld(), ned_transform_.toNeuUU(last_point), ned_transform_.toNeuUU(current_point), color, 0, 3.0F);
        }
        last_point = current_point;
    }

}

msr::airlib::CameraInfo VehicleSimApi::getCameraInfo(int camera_id) const
{
    msr::airlib::CameraInfo camera_info;

    const APIPCamera* camera = getCamera(camera_id);
    camera_info.pose.position = ned_transform_.toNedMeters(camera->GetActorLocation(), true);
    camera_info.pose.orientation = ned_transform_.toQuaternionr(camera->GetActorRotation().Quaternion(), true);
    camera_info.fov = camera->GetCameraComponent()->FieldOfView;
    return camera_info;
}

void VehicleSimApi::setCameraOrientation(int camera_id, const msr::airlib::Quaternionr& orientation)
{
    APIPCamera* camera = getCamera(camera_id);
    FQuat quat = ned_transform_.toFQuat(orientation, true);
    camera->SetActorRelativeRotation(quat);
}

//parameters in NED frame
VehicleSimApi::Pose VehicleSimApi::getPose() const
{
    return toPose(getUUPosition(), getUUOrientation().Quaternion());
}

VehicleSimApi::Pose VehicleSimApi::toPose(const FVector& u_position, const FQuat& u_quat) const
{
    const Vector3r& position = ned_transform_.toNedMeters(u_position);
    const Quaternionr& orientation = ned_transform_.toQuaternionr(u_quat, true);
    return Pose(position, orientation);
}

void VehicleSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    //translate to new VehicleSimApi position & orientation from NED to NEU
    FVector position = ned_transform_.toNeuUU(pose.position);
    state_.current_position = position;

    //quaternion formula comes from http://stackoverflow.com/a/40334755/207661
    FQuat orientation = ned_transform_.toFQuat(pose.orientation, true);

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

void VehicleSimApi::setDebugPose(const Pose& debug_pose)
{
    state_.current_debug_position = ned_transform_.toNeuUU(debug_pose.position);
    if (state_.tracing_enabled && !VectorMath::hasNan(debug_pose.position)) {
        FVector debug_position = state_.current_debug_position - state_.debug_position_offset;
        if ((state_.last_debug_position - debug_position).SizeSquared() > 0.25) {
            UKismetSystemLibrary::DrawDebugLine(pawn_->GetWorld(), state_.last_debug_position, debug_position, FColor(0xaa, 0x33, 0x11), -1, 10.0F);
            UAirBlueprintLib::LogMessage(FString("Debug Pose: "), debug_position.ToCompactString(), LogDebugLevel::Informational);
            state_.last_debug_position = debug_position;
        }
    }
    else if (!state_.tracing_enabled) {
        state_.last_debug_position = state_.current_debug_position - state_.debug_position_offset;
    }
}

bool VehicleSimApi::canTeleportWhileMove()  const
{
    //allow teleportation
    //  if collisions are not enabled
    //  or we have collided but passthrough is enabled
    //     we will flip-flop was_last_move_teleport flag so on one tick we have passthrough and other tick we don't
    //     without flip flopping, collisions can't be detected
    return !state_.collisions_enabled || (state_.collision_info.has_collided && !state_.was_last_move_teleport && state_.passthrough_enabled);
}

void VehicleSimApi::setLogLine(std::string line)
{
    log_line_ = line;
}

std::string VehicleSimApi::getLogLine()
{
    return log_line_;
}






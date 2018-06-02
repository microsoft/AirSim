#include "VehicleSimApi.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "Particles/ParticleSystemComponent.h"
#include "ConstructorHelpers.h"
#include "AirBlueprintLib.h"
#include "common/ClockFactory.hpp"
#include "NedTransform.h"
#include "common/EarthUtils.hpp"

VehicleSimApi::VehicleSimApi(APawn* pawn, const std::map<std::string, APIPCamera*>* cameras, const NedTransform& global_transform,
    const std::string& vehicle_name)
    : pawn_(pawn), cameras_(cameras), vehicle_name_(vehicle_name), ned_transform_(pawn, global_transform)
{
    static ConstructorHelpers::FObjectFinder<UParticleSystem> collision_display(TEXT("ParticleSystem'/AirSim/StarterContent/Particles/P_Explosion.P_Explosion'"));
    if (!collision_display.Succeeded())
        collision_display_template = collision_display.Object;
    else
        collision_display_template = nullptr;

    image_capture_.reset(new UnrealImageCapture(cameras_));

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
    Vector3r nedWrtOrigin = ned_transform_.toGlobalNed(getUUPosition());
    home_geo_point_ = msr::airlib::EarthUtils::nedToGeodetic(nedWrtOrigin, AirSimSettings::singleton().origin_geopoint);

    initial_state_.tracing_enabled = getVehicleSetting().enable_trace;
    initial_state_.collisions_enabled = getVehicleSetting().enable_collisions;
    initial_state_.passthrough_enabled = getVehicleSetting().enable_collision_passthrough;

    initial_state_.collision_info = CollisionInfo();

    initial_state_.was_last_move_teleport = false;
    initial_state_.was_last_move_teleport = canTeleportWhileMove();

    //setup RC
    if (getRemoteControlID() >= 0)
        detectUsbRc();
}

void VehicleSimApi::detectUsbRc()
{
    joystick_.getJoyStickState(getRemoteControlID(), joystick_state_);

    rc_data_.is_initialized = joystick_state_.is_initialized;

    if (rc_data_.is_initialized)
        UAirBlueprintLib::LogMessageString("RC Controller on USB: ", joystick_state_.pid_vid, LogDebugLevel::Informational);
    else
        UAirBlueprintLib::LogMessageString("RC Controller on USB not detected: ",
            std::to_string(joystick_state_.connection_error_code), LogDebugLevel::Informational);
}

void VehicleSimApi::setupCamerasFromSettings()
{
    typedef msr::airlib::AirSimSettings AirSimSettings;

    const auto& vehicle_setting = AirSimSettings::singleton().getVehicleSetting(vehicle_name_);
    const auto& camera_defaults = AirSimSettings::singleton().camera_defaults;

    for (auto& pair : *cameras_) {
        const auto& camera_setting = Utils::findOrDefault(vehicle_setting->cameras, pair.first, camera_defaults);
        APIPCamera* camera = pair.second;
        camera->setupCameraFromSettings(camera_setting, getNedTransform());
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
    state_.collision_info.impact_point = ned_transform_.toLocalNed(Hit.ImpactPoint);
    state_.collision_info.position = ned_transform_.toLocalNed(getUUPosition());
    state_.collision_info.penetration_depth = ned_transform_.toNed(Hit.PenetrationDepth);
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

std::vector<uint8_t> VehicleSimApi::getImage(const std::string& camera_name, ImageCaptureBase::ImageType image_type) const
{
    std::vector<ImageCaptureBase::ImageRequest> request = { ImageCaptureBase::ImageRequest(camera_name, image_type) };
    const std::vector<ImageCaptureBase::ImageResponse>& response = getImages(request);
    if (response.size() > 0)
        return response.at(0).image_data_uint8;
    else
        return std::vector<uint8_t>();
}


msr::airlib::RCData VehicleSimApi::getRCData() const
{
    joystick_.getJoyStickState(getRemoteControlID(), joystick_state_);

    rc_data_.is_valid = joystick_state_.is_valid;

    if (rc_data_.is_valid) {
        //-1 to 1 --> 0 to 1
        rc_data_.throttle = (joystick_state_.left_y + 1) / 2;

        //convert 0 to 1 -> -1 to 1
        rc_data_.yaw = joystick_state_.left_x;
        rc_data_.roll = joystick_state_.right_x;
        rc_data_.pitch = -joystick_state_.right_y;

        //TODO: add fields for z axis?

        //last 8 bits are not used for now
        rc_data_.switch1 = joystick_state_.buttons & 0x0001 ? 1 : 0; //front-upper-left
        rc_data_.switch2 = joystick_state_.buttons & 0x0002 ? 1 : 0; //front-upper-right
        rc_data_.switch3 = joystick_state_.buttons & 0x0004 ? 1 : 0; //top-right-left
        rc_data_.switch4 = joystick_state_.buttons & 0x0008 ? 1 : 0; //top-right-left
        rc_data_.switch5 = joystick_state_.buttons & 0x0010 ? 1 : 0; //top-left-right
        rc_data_.switch6 = joystick_state_.buttons & 0x0020 ? 1 : 0; //top-right-right
        rc_data_.switch7 = joystick_state_.buttons & 0x0040 ? 1 : 0; //top-left-left
        rc_data_.switch8 = joystick_state_.buttons & 0x0080 ? 1 : 0; //top-right-left


        UAirBlueprintLib::LogMessageString("Joystick (T,R,P,Y,Buttons): ", Utils::stringf("%f, %f, %f %f, %d",
            rc_data_.throttle, rc_data_.roll, rc_data_.pitch, rc_data_.yaw, joystick_state_.buttons), LogDebugLevel::Informational);

        //TODO: should below be at controller level info?
        UAirBlueprintLib::LogMessageString("RC Mode: ", rc_data_.switch1 == 0 ? "Angle" : "Rate", LogDebugLevel::Informational);

        UAirBlueprintLib::LogMessage(FString("Joystick (Switches): "), FString::FromInt(joystick_state_.buttons) + ", " +
            FString::FromInt(rc_data_.switch1) + ", " + FString::FromInt(rc_data_.switch2) + ", " + FString::FromInt(rc_data_.switch3) + ", " + FString::FromInt(rc_data_.switch4)
            + ", " + FString::FromInt(rc_data_.switch5) + ", " + FString::FromInt(rc_data_.switch6) + ", " + FString::FromInt(rc_data_.switch7) + ", " + FString::FromInt(rc_data_.switch8),
            LogDebugLevel::Informational);
    }
    //else don't waste time

    return rc_data_;
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
    return getVehicleSetting().rc.remote_control_id;
}

const APIPCamera* VehicleSimApi::getCamera(const std::string& camera_name) const
{
    return cameras_->at(camera_name);
}

APIPCamera* VehicleSimApi::getCamera(const std::string& camera_name)
{
    return const_cast<APIPCamera*>(
        static_cast<const VehicleSimApi*>(this)->getCamera(camera_name));
}

const UnrealImageCapture* VehicleSimApi::getImageCapture() const
{
    return image_capture_.get();
}

int VehicleSimApi::getCameraCount()
{
    return cameras_->size();
}

void VehicleSimApi::reset()
{
    state_ = initial_state_;
    rc_data_ = msr::airlib::RCData();
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
            UKismetSystemLibrary::DrawDebugLine(pawn_->GetWorld(), ned_transform_.fromLocalNed(last_point), ned_transform_.fromLocalNed(current_point), color, 0, 3.0F);
        }
        last_point = current_point;
    }

}

msr::airlib::CameraInfo VehicleSimApi::getCameraInfo(const std::string& camera_name) const
{
    msr::airlib::CameraInfo camera_info;

    const APIPCamera* camera = getCamera(camera_name);
    camera_info.pose.position = ned_transform_.toLocalNed(camera->GetActorLocation());
    camera_info.pose.orientation = ned_transform_.toNed(camera->GetActorRotation().Quaternion());
    camera_info.fov = camera->GetCameraComponent()->FieldOfView;
    return camera_info;
}

void VehicleSimApi::setCameraOrientation(const std::string& camera_name, const msr::airlib::Quaternionr& orientation)
{
    APIPCamera* camera = getCamera(camera_name);
    FQuat quat = ned_transform_.fromNed(orientation);
    camera->SetActorRelativeRotation(quat);
}

//parameters in NED frame
VehicleSimApi::Pose VehicleSimApi::getPose() const
{
    return toPose(getUUPosition(), getUUOrientation().Quaternion());
}

VehicleSimApi::Pose VehicleSimApi::toPose(const FVector& u_position, const FQuat& u_quat) const
{
    const Vector3r& position = ned_transform_.toLocalNed(u_position);
    const Quaternionr& orientation = ned_transform_.toNed(u_quat);
    return Pose(position, orientation);
}

void VehicleSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    //translate to new VehicleSimApi position & orientation from NED to NEU
    FVector position = ned_transform_.fromLocalNed(pose.position);
    state_.current_position = position;

    //quaternion formula comes from http://stackoverflow.com/a/40334755/207661
    FQuat orientation = ned_transform_.fromNed(pose.orientation);

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
    state_.current_debug_position = ned_transform_.fromLocalNed(debug_pose.position);
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

std::string VehicleSimApi::getLogLine() const
{
    return log_line_;
}






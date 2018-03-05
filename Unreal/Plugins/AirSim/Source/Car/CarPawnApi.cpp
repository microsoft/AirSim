#include "CarPawnApi.h"
#include "AirBlueprintLib.h"


CarPawnApi::CarPawnApi(VehiclePawnWrapper* pawn, UWheeledVehicleMovementComponent* movement)
    : pawn_(pawn), movement_(movement)
{
}

std::vector<ImageCaptureBase::ImageResponse> CarPawnApi::simGetImages(
    const std::vector<ImageCaptureBase::ImageRequest>& requests)
{
    std::vector<ImageCaptureBase::ImageResponse> responses;

    ImageCaptureBase* camera = pawn_->getImageCapture();
    camera->getImages(requests, responses);

    return responses;
}

bool CarPawnApi::simSetSegmentationObjectID(const std::string& mesh_name, int object_id, 
    bool is_name_regex)
{
    bool success;
    UAirBlueprintLib::RunCommandOnGameThread([mesh_name, object_id, is_name_regex, &success]() {
        success = UAirBlueprintLib::SetMeshStencilID(mesh_name, object_id, is_name_regex);
    }, true);
    return success;
}

void CarPawnApi::simPrintLogMessage(const std::string& message, const std::string& message_param, unsigned char severity)
{
    pawn_->printLogMessage(message, message_param, severity);
}

int CarPawnApi::simGetSegmentationObjectID(const std::string& mesh_name)
{
    return UAirBlueprintLib::GetMeshStencilID(mesh_name);
}

msr::airlib::CollisionInfo CarPawnApi::getCollisionInfo()
{
    return pawn_->getCollisionInfo();
}

std::vector<uint8_t> CarPawnApi::simGetImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type)
{
    std::vector<ImageCaptureBase::ImageRequest> request = { ImageCaptureBase::ImageRequest(camera_id, image_type) };
    const std::vector<ImageCaptureBase::ImageResponse>& response = simGetImages(request);
    if (response.size() > 0)
        return response.at(0).image_data_uint8;
    else
        return std::vector<uint8_t>();
}

void CarPawnApi::setCarControls(const CarApiBase::CarControls& controls)
{
    if (api_control_enabled_)
        last_controls_ = controls;
    //else don't save

    if (!controls.is_manual_gear && movement_->GetTargetGear() < 0)
        movement_->SetTargetGear(0, true); //in auto gear we must have gear >= 0
    if (controls.is_manual_gear && movement_->GetTargetGear() != controls.manual_gear)
        movement_->SetTargetGear(controls.manual_gear, controls.gear_immediate);

    movement_->SetThrottleInput(controls.throttle);
    movement_->SetSteeringInput(controls.steering);
    movement_->SetBrakeInput(controls.brake);
    movement_->SetHandbrakeInput(controls.handbrake);
    movement_->SetUseAutoGears(!controls.is_manual_gear);
}

msr::airlib::Pose CarPawnApi::simGetObjectPose(const std::string& actor_name)
{
    msr::airlib::Pose pose;

    UAirBlueprintLib::RunCommandOnGameThread([&pose, &actor_name, this]() {
        pose = pawn_->getActorPose(actor_name);
    }, true);

    return pose;
}

const CarApiBase::CarControls& CarPawnApi::getCarControls() const
{
    return last_controls_;
}

msr::airlib::CameraInfo CarPawnApi::getCameraInfo(int camera_id) const
{
    return pawn_->getCameraInfo(camera_id);
}

void CarPawnApi::setCameraOrientation(int camera_id, const msr::airlib::Quaternionr& orientation)
{
    UAirBlueprintLib::RunCommandOnGameThread([&camera_id, &orientation, this]() {
        pawn_->setCameraOrientation(camera_id, orientation);
    }, true);
}

CarApiBase::CarState CarPawnApi::getCarState()
{
    CarApiBase::CarState state(
        movement_->GetForwardSpeed() / 100, //cm/s -> m/s
        movement_->GetCurrentGear(),
        pawn_->getCollisionInfo(),
        *pawn_->getTrueKinematics(),
        msr::airlib::ClockFactory::get()->nowNanos()
    );
    return state;
}

void CarPawnApi::reset()
{
    last_controls_ = CarControls();
    auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_->getPawn());
    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        pawn_->reset();
        for (auto* phys_comp : phys_comps) {
            phys_comp->SetPhysicsAngularVelocity(FVector::ZeroVector);
            phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
            phys_comp->SetSimulatePhysics(false);
        }
        //movement_->ResetMoveState();
        //movement_->SetActive(false);
        //movement_->SetActive(true, true);
        setCarControls(CarControls());
        
    }, true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps)
            phys_comp->SetSimulatePhysics(true);
    }, true);
}

void CarPawnApi::simSetPose(const msr::airlib::Pose& pose, bool ignore_collision)
{
    UAirBlueprintLib::RunCommandOnGameThread([this, pose, ignore_collision]() {
        pawn_->setPose(pose, ignore_collision);
    }, true);
}

msr::airlib::Pose CarPawnApi::simGetPose()
{
    return pawn_->getPose();
}

msr::airlib::GeoPoint CarPawnApi::getHomeGeoPoint()
{
    return pawn_->getHomePoint();
}

void CarPawnApi::enableApiControl(bool is_enabled)
{
    if (api_control_enabled_ != is_enabled) {
        last_controls_ = CarControls();
        api_control_enabled_ = is_enabled;
    }
}

bool CarPawnApi::isApiControlEnabled() const
{
    return api_control_enabled_;
}

CarPawnApi::~CarPawnApi() = default;

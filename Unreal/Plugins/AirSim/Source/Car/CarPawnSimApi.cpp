#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "CarPawnApi.h"
#include <exception>

using namespace msr::airlib;

CarPawnSimApi::CarPawnSimApi(APawn* pawn, const NedTransform& global_transform, CollisionSignal& collision_signal,
    const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras, UClass* pip_camera_class, UParticleSystem* collision_display_template,
    const CarPawnApi::CarControls&  keyboard_controls,
    UWheeledVehicleMovementComponent* movement, const msr::airlib::GeoPoint& home_geopoint)
    : PawnSimApi(pawn, global_transform, collision_signal, cameras, pip_camera_class, collision_display_template), 
      keyboard_controls_(keyboard_controls)
{
    createVehicleApi(movement, home_geopoint);

    //TODO: should do reset() here?
    joystick_controls_ = CarPawnApi::CarControls();

    Environment::State initial_environment;
    initial_environment.position = getPose().position;
    initial_environment.geo_point = home_geopoint;
    environment_.reset(new Environment(initial_environment));
}

void CarPawnSimApi::createVehicleApi(UWheeledVehicleMovementComponent* movement, const msr::airlib::GeoPoint& home_geopoint)
{
    //create vehicle params
    //std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_api_ = std::unique_ptr<CarApiBase>(new CarPawnApi(movement, home_geopoint));
}

std::string CarPawnSimApi::getLogLine() const
{
    const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();
    const auto state = vehicle_api_->getCarState();
    uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

    //TODO: because this bug we are using alternative code with stringstream
    //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html

    std::string line;
    line.append(std::to_string(timestamp_millis)).append("\t")
        .append(std::to_string(kinematics->pose.position.x())).append("\t")
        .append(std::to_string(kinematics->pose.position.y())).append("\t")
        .append(std::to_string(kinematics->pose.position.z())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.w())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.x())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.y())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.z())).append("\t")
        .append(std::to_string(current_controls_.throttle)).append("\t")
        .append(std::to_string(current_controls_.steering)).append("\t")
        .append(std::to_string(current_controls_.brake)).append("\t")
        .append(std::to_string(state.gear)).append("\t")
        .append(std::to_string(state.handbrake)).append("\t")
        .append(std::to_string(state.rpm)).append("\t")
        .append(std::to_string(state.speed)).append("\t")
        ;

    return line;

    //std::stringstream ss;
    //ss << timestamp_millis << "\t";
    //ss << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
    //ss << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
    //ss << "\n";
    //return ss.str();

}

const msr::airlib::Kinematics::State* CarPawnSimApi::getGroundTruthKinematics() const
{
    return &kinematics_;
}
const msr::airlib::Environment* CarPawnSimApi::getGroundTruthEnvironment() const
{
    return environment_.get();
}

void CarPawnSimApi::updateKinematics(float dt)
{
    const auto last_kinematics = kinematics_;

    kinematics_.pose = getPose();
    kinematics_.twist.linear = getNedTransform().toLocalNed(getPawn()->GetVelocity());
    kinematics_.twist.angular = msr::airlib::VectorMath::toAngularVelocity(
        kinematics_.pose.orientation, last_kinematics.pose.orientation, dt);

    kinematics_.accelerations.linear = (kinematics_.twist.linear - last_kinematics.twist.linear) / dt;
    kinematics_.accelerations.angular = (kinematics_.twist.angular - last_kinematics.twist.angular) / dt;

    //TODO: update other fields

}

void CarPawnSimApi::updateRenderedState(float dt)
{
    updateKinematics(dt);
    
    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    //TODO: do we need this for cars?
    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}

void CarPawnSimApi::updateRendering(float dt)
{
    updateCarControls();

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

void CarPawnSimApi::updateCarControls()
{
    auto rc_data = getRCData();

    if (rc_data.is_initialized) {
        if (!rc_data.is_valid) {
            UAirBlueprintLib::LogMessageString("Control Mode: ", "[INVALID] Wheel/Joystick", LogDebugLevel::Informational);
            return;
        }
        UAirBlueprintLib::LogMessageString("Control Mode: ", "Wheel/Joystick", LogDebugLevel::Informational);

        //TODO: move this to SimModeBase?
        //if ((joystick_state_.buttons & 4) | (joystick_state_.buttons & 1024)) { //X button or Start button
        //    reset();
        //    return;
        //}

        // Thrustmaster devices
        if (rc_data.vendor_id == "VID_044F") {
            joystick_controls_.steering = rc_data.yaw;
            joystick_controls_.throttle = (-rc_data.right_z + 1) / 2;
            joystick_controls_.brake = rc_data.throttle;

            auto car_state = vehicle_api_->getCarState();
            float rumble_strength = 0.66 + (car_state.rpm
                / car_state.maxrpm) / 3;
            float auto_center = (1.0 - 1.0 / (std::abs(car_state.speed / 120) + 1.0))
            * (rc_data.yaw / 3);
            setRCForceFeedback(rumble_strength, auto_center);
        }
        // Anything else, typically Logitech G920 wheel
        else {
            joystick_controls_.steering = (rc_data.throttle * 2 - 1) * 1.25;
            joystick_controls_.throttle = (-rc_data.roll + 1) / 2;
            joystick_controls_.brake = -rc_data.right_z + 1;
        }
        //Two steel levers behind wheel
        joystick_controls_.handbrake = (rc_data.getSwitch(5)) | (rc_data.getSwitch(6)) ? 1 : 0;

        if ((rc_data.getSwitch(8)) | (rc_data.getSwitch(1))) { //RSB button or B button
            joystick_controls_.manual_gear = -1;
            joystick_controls_.is_manual_gear = true;
            joystick_controls_.gear_immediate = true;
        }
        else if ((rc_data.getSwitch(9)) | (rc_data.getSwitch(0))) { //LSB button or A button
            joystick_controls_.manual_gear = 0;
            joystick_controls_.is_manual_gear = false;
            joystick_controls_.gear_immediate = true;
        }

        current_controls_ = joystick_controls_;
    }
    else {
        UAirBlueprintLib::LogMessageString("Control Mode: ", "Keyboard", LogDebugLevel::Informational);
        current_controls_ = keyboard_controls_;
    }

    //if API-client control is not active then we route keyboard/joystick control to car
    if (!vehicle_api_->isApiControlEnabled()) {
        //all car controls from anywhere must be routed through API component
        vehicle_api_->setCarControls(current_controls_);
    }
    else {
        UAirBlueprintLib::LogMessageString("Control Mode: ", "API", LogDebugLevel::Informational);
        current_controls_ = vehicle_api_->getCarControls();
    }
    UAirBlueprintLib::LogMessageString("Accel: ", std::to_string(current_controls_.throttle), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Break: ", std::to_string(current_controls_.brake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Steering: ", std::to_string(current_controls_.steering), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Handbrake: ", std::to_string(current_controls_.handbrake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Target Gear: ", std::to_string(current_controls_.manual_gear), LogDebugLevel::Informational);
}


//*** Start: UpdatableState implementation ***//
void CarPawnSimApi::reset()
{
    vehicle_api_->reset();
    PawnSimApi::reset();
    environment_->reset();
}

void CarPawnSimApi::update()
{
    PawnSimApi::update();
    //update position from kinematics so we have latest position after physics update
    environment_->setPosition(kinematics_.pose.position);
    environment_->update();
}

void CarPawnSimApi::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = getUUPosition();
    reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
}
//*** End: UpdatableState implementation ***//


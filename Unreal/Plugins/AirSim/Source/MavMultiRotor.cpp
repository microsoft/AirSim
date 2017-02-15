#include "AirSim.h"
#include "MavMultiRotor.h"
#include "AirBlueprintLib.h"
#include "Settings.h"

void MavMultiRotor::initialize(AFlyingPawn* vehicle_pawn)
{
    vehicle_pawn_ = vehicle_pawn;
    vehicle_pawn_->initialize();

    //init vehicle
    auto initial_kinematics = Kinematics::State::zero();
    initial_kinematics.pose = vehicle_pawn_->getPose();
    msr::airlib::Environment::State initial_environment;
    initial_environment.position = initial_kinematics.pose.position;
    initial_environment.geo_point = vehicle_pawn_->getHomePoint();
    initial_environment.min_z_over_ground = vehicle_pawn_->getMinZOverGround();
    environment_.initialize(initial_environment);
    vehicle_.initialize(Px4QuadX::Params(), initial_kinematics, 
        &environment_, static_cast<ControllerBase*>(&mav_));

    mav_.initialize(&vehicle_);
}

void MavMultiRotor::beginPlay()
{
    openConnection();
}

void MavMultiRotor::endPlay()
{
    closeConnection();
}

msr::airlib::MavLinkHelper::HILConnectionInfo MavMultiRotor::getConnectionInfo()
{
    auto connection_info = vehicle_pawn_->getHILConnectionInfo();

    Settings& settings = Settings::singleton();

    auto settings_filename = Settings::singleton().getFileName();
    if (!settings_filename.empty()) {
        std::wstring msg = L"Loading settings from " + settings_filename;
        UAirBlueprintLib::LogMessage(FString(msg.c_str()), TEXT(""), LogDebugLevel::Success, 180);

        Settings child = settings.getChild(connection_info.vehicle_name);
        // allow json overrides on a per-vehicle basis.
        connection_info.use_serial = child.getBool("UseSerial", connection_info.use_serial);
        connection_info.ip_address = child.getString("UdpIp", connection_info.ip_address);
        connection_info.ip_port = child.getInt("UdpPort", connection_info.ip_port);
        connection_info.serial_port = child.getString("SerialPort", connection_info.serial_port);
        connection_info.baud_rate = child.getInt("SerialBaudRate", connection_info.baud_rate);
    }

    //BUG: writing this back causes 2nd run to use old setting but not the 3rd run
    //if (connection_info.vehicle_name.size() > 0) {
    //    child.setBool("UseSerial", clone.use_serial);
    //    child.setString("UdpIp", clone.ip_address);
    //    child.setInt("UdpPort", clone.ip_port);
    //    child.setString("SerialPort", clone.serial_port);
    //    child.setInt("SerialBaudRate", clone.baud_rate);
    //    settings.setChild(connection_info.vehicle_name, child);
    //    settings.saveJSonFile(L"settings.json");
    //}

    return connection_info;
}

void MavMultiRotor::openConnection()
{
    //connect to HIL
    try {

        mav_.connectToHIL(getConnectionInfo());
    }
    catch (std::runtime_error ex) {

        UAirBlueprintLib::LogMessage(FString("Connection to drone failed, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
        UAirBlueprintLib::LogMessage(FString(ex.what()), TEXT(""), LogDebugLevel::Failure, 180);
    }

    try {
        if (!mav_.connectToLogViewer()) {
            UAirBlueprintLib::LogMessage(FString("Connection to LogViewer failed, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
        }
    }
    catch (std::runtime_error ex) {

        UAirBlueprintLib::LogMessage(FString("Connection to LogViewer failed, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
    }

    try {
        if (!mav_.connectToQGC()) {
            UAirBlueprintLib::LogMessage(FString("Connection to QGroundControl failed, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
        }
    }
    catch (std::runtime_error ex) {

        UAirBlueprintLib::LogMessage(FString("Connection to QGroundControl failed, please check your settings.json"), TEXT(""), LogDebugLevel::Failure, 180);
    }
}

void MavMultiRotor::closeConnection()
{
    mav_.close();
}

void MavMultiRotor::updateRenderedState()
{
    //move collison info from rendering engine to vehicle
    vehicle_.setCollisionInfo(vehicle_pawn_->getCollisonInfo());
    //update ground level
    environment_.getState().min_z_over_ground = vehicle_pawn_->getMinZOverGround();
    //update pose of object in rendering engine
    vehicle_pawn_->setPose(vehicle_.getPose());

    //update rotor poses
    for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
        const auto& rotor_output = vehicle_.getRotorOutput(i);
        rotor_speeds_[i] = rotor_output.speed;
        rotor_directions_[i] = rotor_output.turning_direction;
        rotor_thrusts_[i] = rotor_output.thrust;
        rotor_controls_filtered_[i] = rotor_output.control_signal_filtered;
    }

    //retrieve MavLink status messages
    mav_.getStatusMessages(mav_messages_);
}

void MavMultiRotor::updateRendering()
{
    //update rotor animations
    for (unsigned int i = 0; i < vehicle_.vertexCount(); ++i) {
        vehicle_pawn_->setRotorSpeed(i, rotor_speeds_[i] * rotor_directions_[i]);
    }

    for (auto i = 0; i < mav_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(mav_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }
}


//*** Start: UpdatableState implementation ***//
void MavMultiRotor::reset()
{
    vehicle_pawn_->reset();    //we do flier resetPose so that flier is placed back without collisons
    vehicle_.reset();
}

void MavMultiRotor::update(real_T dt)
{
    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    vehicle_.update(dt);
}

void MavMultiRotor::reportState(StateReporter& reporter)
{
    vehicle_.reportState(reporter);
}
   
MavMultiRotor::UpdatableObject* MavMultiRotor::getPhysicsBody()
{
    return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

msr::airlib::DroneControlBase* MavMultiRotor::createOrGetDroneControl()
{
    return mav_.createOrGetDroneControl();
}

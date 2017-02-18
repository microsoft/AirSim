#include "AirSim.h"
#include "SimModeWorldMultiRotor.h"
#include "AirBlueprintLib.h"
#include "control/DroneControlBase.hpp"
#include "physics/PhysicsBody.hpp"
#include <memory>
#include "control/Settings.h"
#include "FlyingPawn.h"

void ASimModeWorldMultiRotor::BeginPlay() {
    Super::BeginPlay();

    if (fpv_vehicle_ != nullptr) {
        //create its control server
        drone_control_server_.reset(new msr::airlib::DroneControlServer(fpv_vehicle_->createOrGetDroneControl()));
        std::string server_address = Settings::singleton().getString("LocalHostIp", "127.0.0.1");
        rpclib_server_.reset(new msr::airlib::RpcLibServer(drone_control_server_.get(), server_address));
        rpclib_server_->start();
    }
}

void ASimModeWorldMultiRotor::Tick(float DeltaSeconds) {
    if (fpv_vehicle_ != nullptr) {
        using namespace msr::airlib;
        auto camera_type = drone_control_server_->getImageTypeForCamera(0);
        if (camera_type != DroneControlBase::ImageType::None) {
            if (CameraDirector != nullptr) {
                APIPCamera* camera = CameraDirector->getCamera(0);
                EPIPCameraType pip_type;
                if (camera != nullptr) {
                    //TODO: merge these two different types?
                    switch (camera_type) {
                    case DroneControlBase::ImageType::Scene:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SCENE;
                        break;
                    case DroneControlBase::ImageType::Depth:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_DEPTH;
                        break;
                    case DroneControlBase::ImageType::Segmentation:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SEG;
                        break;
                    default:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_NONE;
                    }
                    float width, height;
                    image_.Empty();
                    camera->getScreenshot(pip_type, image_, width, height);
                    drone_control_server_->setImageForCamera(0, camera_type, std::vector<msr::airlib::uint8_t>(image_.GetData(), image_.GetData() + image_.Num()));
                }
            }
        }

        if (isRecording() && record_file.is_open()) {
            auto physics_body = static_cast<msr::airlib::PhysicsBody*>(fpv_vehicle_->getPhysicsBody());
            auto kinematics = physics_body->getKinematics();

            record_file << msr::airlib::Utils::getTimeSinceEpochMillis() << "\t";    //TODO: maintain simulation timer instead
            record_file << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z()  << "\t";
            record_file << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z()  << "\t";
            record_file << "\n";
            if (CameraDirector != nullptr) {
                APIPCamera* camera = CameraDirector->getCamera(0);
                if (camera != nullptr) {
                    camera->saveScreenshot(EPIPCameraType::PIP_CAMERA_TYPE_SCENE, record_folder_path.c_str(), record_tick_count);
                }
            }
        }
    }

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldMultiRotor::EndPlay(const EEndPlayReason::Type EndPlayReason) {
    if (fpv_vehicle_ != nullptr) {
        rpclib_server_->stop();
        rpclib_server_.release();
        drone_control_server_.release();
    }

    Super::EndPlay(EndPlayReason);
}

bool ASimModeWorldMultiRotor::checkConnection() {
    msr::airlib::MavLinkHelper mav;
    if (mav.findPixhawk() == "") {
        if (!Settings::singleton().isLoadSuccess())
            Settings::singleton().loadJSonFile(L"settings.json");
        if (!Settings::singleton().isLoadSuccess()) {
            UAirBlueprintLib::LogMessage(TEXT("Could not detect Pixhawk on any serial ports"), TEXT(""), LogDebugLevel::Failure);
            UAirBlueprintLib::LogMessage(TEXT("Either connect Pixhawk or use software-in-loop mode"), TEXT(""), LogDebugLevel::Failure);
            UAirBlueprintLib::LogMessage("Settings can also be set at: ", Settings::singleton().getFileName().c_str(), LogDebugLevel::Failure);
            UAirBlueprintLib::LogMessage(TEXT("Instructions are available at: "), TEXT("http://github.com/Microsoft/AirSim"), LogDebugLevel::Failure);
            return false;
        }
    }

    return true;
}

void ASimModeWorldMultiRotor::createVehicles(std::vector<VehiclePtr>& vehicles) {
    if (!checkConnection())
        return;

    //get FPV drone
    AActor* fpv_drone = nullptr;
    if (CameraDirector != nullptr) {
        fpv_drone = CameraDirector->TargetPawn;
    }

    //detect vehicles in the project and add them in simulation
    TArray<AActor*> pawns;
    UAirBlueprintLib::FindAllActor<AFlyingPawn>(this, pawns);
    for (AActor* pawn : pawns) {
        auto vehicle = std::make_shared<MavMultiRotor>();
        vehicle->initialize(static_cast<AFlyingPawn*>(pawn));
        vehicles.push_back(std::static_pointer_cast<VehicleBase>(vehicle));

        if (pawn == fpv_drone) {
            fpv_vehicle_ = vehicle;
        }
    }
}



#include "AirSim.h"
#include "SimModeWorldMultiRotor.h"
#include "AirBlueprintLib.h"
#include "controllers/DroneControllerBase.hpp"
#include "physics/PhysicsBody.hpp"
#include <memory>
#include "FlyingPawn.h"
#include "Logging/MessageLog.h"
#include "vehicles/MultiRotorParamsFactory.hpp"
#include "common/common_utils/Log.hpp"

using namespace common_utils;

class ASimLog : public Log
{
    FMessageLog log;

public:
    ASimLog() : log("BlueprintLog") {
    }

    virtual void logMessage(const char* message) override {
        log.Info(FText::FromString(FString(message)));
    }

    virtual void logError(const char* message) override {
        log.Error(FText::FromString(FString(message)));
    }
};

static ASimLog GlobalASimLog;

void ASimModeWorldMultiRotor::BeginPlay()
{
    Super::BeginPlay();

    Log::setLog(&GlobalASimLog);

    if (fpv_vehicle_connector_ != nullptr) {
        //create its control server
        try {
            fpv_vehicle_connector_->startApiServer();
        }
        catch (std::exception& ex) {
            UAirBlueprintLib::LogMessage("Cannot start RpcLib Server",  ex.what(), LogDebugLevel::Failure);
        }
    }
}

void ASimModeWorldMultiRotor::Tick(float DeltaSeconds)
{
    if (fpv_vehicle_connector_ != nullptr && fpv_vehicle_connector_->isApiServerStarted() && getVehicleCount() > 0) {

        using namespace msr::airlib;
        auto controller = static_cast<DroneControllerBase*>(fpv_vehicle_connector_->getController());
        auto camera_type = controller->getImageTypeForCamera(0);
        if (camera_type != DroneControllerBase::ImageType::None) { 
            if (CameraDirector != nullptr) {
                APIPCamera* camera = CameraDirector->getCamera(0);
                EPIPCameraType pip_type;
                if (camera != nullptr) {
                    //TODO: merge these two different types?
                    switch (camera_type) {
                    case DroneControllerBase::ImageType::Scene:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SCENE; break;
                    case DroneControllerBase::ImageType::Depth:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_DEPTH; break;
                    case DroneControllerBase::ImageType::Segmentation:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_SEG; break;
                    default:
                        pip_type = EPIPCameraType::PIP_CAMERA_TYPE_NONE;
                    }
                    float width, height;
                    image_.Empty();
                    camera->getScreenshot(pip_type, image_, width, height);
                    controller->setImageForCamera(0, camera_type, std::vector<uint8_t>(image_.GetData(), image_.GetData() + image_.Num()));
                }
            }
        }

        if (isRecording() && record_file.is_open()) {
            if (!isLoggingStarted)
            {
                FString imagePathPrefix = common_utils::FileSystem::getLogFileNamePath("img_", "", "", false).c_str();
                FRecordingThread::ThreadInit(imagePathPrefix, this);
                isLoggingStarted = true;
            }
        }

        if (!isRecording() && isLoggingStarted)
        {
            FRecordingThread::Shutdown();
            isLoggingStarted = false;
        }
    }

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldMultiRotor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (fpv_vehicle_connector_ != nullptr) {
        fpv_vehicle_connector_->stopApiServer();
    }

    if (isLoggingStarted)
    {
        FRecordingThread::Shutdown();
        isLoggingStarted = false;
    }

    Super::EndPlay(EndPlayReason);
}

bool ASimModeWorldMultiRotor::checkConnection()
{
    return true;
}

void ASimModeWorldMultiRotor::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    if (!checkConnection())
        return;

    //get FPV drone
    AActor* fpv_pawn = nullptr;
    if (CameraDirector != nullptr) {
        fpv_pawn = CameraDirector->TargetPawn;
    }

    //detect vehicles in the project and add them in simulation
    TArray<AActor*> pawns;
    UAirBlueprintLib::FindAllActor<AFlyingPawn>(this, pawns);
    for (AActor* pawn : pawns) {
        auto vehicle = createVehicle(static_cast<AFlyingPawn*>(pawn));
        if (vehicle != nullptr) {
            vehicles.push_back(vehicle);

            if (pawn == fpv_pawn) {
                fpv_vehicle_connector_ = vehicle;
            }
        }
        //else we don't have vehicle for this pawn
    }
}

ASimModeWorldBase::VehiclePtr ASimModeWorldMultiRotor::createVehicle(AFlyingPawn* pawn)
{
    vehicle_params_ = MultiRotorParamsFactory::createConfig(pawn->getVehicleName());

    auto vehicle = std::make_shared<MultiRotorConnector>();
    vehicle->initialize(pawn, vehicle_params_.get());
    return std::static_pointer_cast<VehicleConnectorBase>(vehicle);
}



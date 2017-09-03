#include "SimModeWorldMultiRotor.h"
#include "ConstructorHelpers.h"
#include "AirBlueprintLib.h"
#include "controllers/DroneControllerBase.hpp"
#include "physics/PhysicsBody.hpp"
#include <memory>
#include "Logging/MessageLog.h"
#include "vehicles/MultiRotorParamsFactory.hpp"


ASimModeWorldMultiRotor::ASimModeWorldMultiRotor()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<TMultiRotorPawn> vehicle_pawn_class(TEXT("Blueprint'/AirSim/Blueprints/BP_FlyingPawn'"));
    vehicle_pawn_class_ = vehicle_pawn_class.Succeeded() ? vehicle_pawn_class.Class : nullptr;
}

void ASimModeWorldMultiRotor::BeginPlay()
{
    Super::BeginPlay();

    if (fpv_vehicle_connector_ != nullptr) {
        //create its control server
        try {
            fpv_vehicle_connector_->startApiServer();
        }
        catch (std::exception& ex) {
            UAirBlueprintLib::LogMessageString("Cannot start RpcLib Server",  ex.what(), LogDebugLevel::Failure);
        }
    }
}

void ASimModeWorldMultiRotor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismental
    stopAsyncUpdator();

    if (fpv_vehicle_connector_ != nullptr) {
        fpv_vehicle_connector_->stopApiServer();
    }

    if (isLoggingStarted)
    {
        FRecordingThread::Shutdown();
        isLoggingStarted = false;
    }

    //for (AActor* actor : spawned_actors_) {
    //    actor->Destroy();
    //}
    spawned_actors_.Empty();
    if (CameraDirector != nullptr) {
        fpv_vehicle_connector_ = nullptr;
        CameraDirector = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

AVehiclePawnBase* ASimModeWorldMultiRotor::getFpvVehiclePawn()
{
    return fpv_vehicle_pawn_;
}

void ASimModeWorldMultiRotor::setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles)
{
    //get player controller
    APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
    FTransform actor_transform = player_controller->GetActorTransform();
    //put camera little bit above vehicle
    FTransform camera_transform(actor_transform.GetLocation() + FVector(-300, 0, 200));

    //we will either find external camera if it already exist in evironment or create one
    APIPCamera* external_camera;

    //find all BP camera directors in the environment
    {
        TArray<AActor*> camera_dirs;
        UAirBlueprintLib::FindAllActor<ACameraDirector>(this, camera_dirs);
        if (camera_dirs.Num() == 0) {
            //create director
            FActorSpawnParameters camera_spawn_params;
            camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            CameraDirector = this->GetWorld()->SpawnActor<ACameraDirector>(camera_director_class_, camera_transform, camera_spawn_params);
            spawned_actors_.Add(CameraDirector);

            //create external camera required for the director
            external_camera = this->GetWorld()->SpawnActor<APIPCamera>(external_camera_class_, camera_transform, camera_spawn_params);
            spawned_actors_.Add(external_camera);
        }
        else {
            CameraDirector = static_cast<ACameraDirector*>(camera_dirs[0]);
            external_camera = CameraDirector->getExternalCamera();
        }
    }

    //find all vehicle pawns
    {
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<TMultiRotorPawn>(this, pawns);

        //if no vehicle pawns exists in environment
        if (pawns.Num() == 0) {
            //create vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.SpawnCollisionHandlingOverride = 
                ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            TMultiRotorPawn* spawned_pawn = this->GetWorld()->SpawnActor<TMultiRotorPawn>(
                vehicle_pawn_class_, actor_transform, pawn_spawn_params);

            spawned_pawn->IsFpvVehicle = true;

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);
        }

        //set up vehicle pawns
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TMultiRotorPawn* vehicle_pawn = static_cast<TMultiRotorPawn*>(pawn);
            if (enable_collision_passthrough)
                vehicle_pawn->EnablePassthroughOnCollisons = true;    
            vehicle_pawn->initializeForBeginPlay();

            //chose first pawn as FPV if none is designated as FPV
            if (vehicle_pawn->IsFpvVehicle || fpv_vehicle_pawn_ == nullptr)
                fpv_vehicle_pawn_ = vehicle_pawn;

            //now create the connector for each pawn
            auto vehicle = createVehicle(vehicle_pawn);
            if (vehicle != nullptr) {
                vehicles.push_back(vehicle);

                if (vehicle_pawn == fpv_vehicle_pawn_)
                    fpv_vehicle_connector_ = vehicle;
            }
            //else we don't have vehicle for this pawn
        }
    }

    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_pawn_, external_camera);
}

void ASimModeWorldMultiRotor::Tick(float DeltaSeconds)
{
    if (fpv_vehicle_connector_ != nullptr && fpv_vehicle_connector_->isApiServerStarted() && getVehicleCount() > 0) {

        if (isRecording() && getRecordingFile().isRecording()) {
            if (!isLoggingStarted)
            {
                FRecordingThread::ThreadInit(fpv_vehicle_connector_->getCamera(), & getRecordingFile(), 
                    static_cast<msr::airlib::PhysicsBody*>(fpv_vehicle_connector_->getPhysicsBody()), recording_settings);
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

bool ASimModeWorldMultiRotor::checkConnection()
{
    return true;
}

void ASimModeWorldMultiRotor::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    if (!checkConnection())
        return;

    //find vehicles and cameras available in environment
    //if none available then we will create one
    setupVehiclesAndCamera(vehicles);
}

ASimModeWorldBase::VehiclePtr ASimModeWorldMultiRotor::createVehicle(AFlyingPawn* vehicle_pawn)
{
    auto vehicle_params = MultiRotorParamsFactory::createConfig(
        vehicle_pawn->VehicleConfigName == "" ? default_vehicle_config 
        : std::string(TCHAR_TO_UTF8(* vehicle_pawn->VehicleConfigName)));

	vehicle_params_.push_back(std::move(vehicle_params));

    auto vehicle = std::make_shared<MultiRotorConnector>(
        vehicle_pawn, vehicle_params_.back().get(), enable_rpc, api_server_address, 
        vehicle_params_.back()->getParams().api_server_port, manual_pose_controller);
    return std::static_pointer_cast<VehicleConnectorBase>(vehicle);
}



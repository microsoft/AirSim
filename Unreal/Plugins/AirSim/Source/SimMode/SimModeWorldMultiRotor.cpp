#include "AirSim.h"
#include "SimModeWorldMultiRotor.h"
#include "AirBlueprintLib.h"
#include "controllers/DroneControllerBase.hpp"
#include "physics/PhysicsBody.hpp"
#include <memory>
#include "FlyingPawn.h"
#include "Logging/MessageLog.h"
#include "vehicles/MultiRotorParamsFactory.hpp"


ASimModeWorldMultiRotor::ASimModeWorldMultiRotor()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<AVehiclePawnBase> vehicle_pawn_class(TEXT("Blueprint'/AirSim/Blueprints/BP_FlyingPawn'"));
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

AVehiclePawnBase* ASimModeWorldMultiRotor::getFpvVehiclePawn()
{
    return fpv_vehicle_pawn_;
}

void ASimModeWorldMultiRotor::setupVehiclesAndCamera()
{
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    FTransform actor_transform = controller->GetActorTransform();
    //put camera little bit above vehicle
    FTransform camera_transform(actor_transform.GetLocation() + FVector(-300, 0, 200));

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

    {
        //find all vehicle pawns
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<AVehiclePawnBase>(this, pawns);
        if (pawns.Num() == 0) {
            //create vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            fpv_vehicle_pawn_ = this->GetWorld()->SpawnActor<AVehiclePawnBase>(vehicle_pawn_class_, actor_transform, pawn_spawn_params);
            spawned_actors_.Add(fpv_vehicle_pawn_);
        }
        else {
            fpv_vehicle_pawn_ = static_cast<AVehiclePawnBase*>(pawns[0]);
        }
    }

    if (usage_scenario == kUsageScenarioComputerVision)
        fpv_vehicle_pawn_->EnablePassthroughOnCollisons = true;
    fpv_vehicle_pawn_->initializeForBeginPlay();
    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_pawn_, external_camera);
}

void ASimModeWorldMultiRotor::Tick(float DeltaSeconds)
{
    if (fpv_vehicle_connector_ != nullptr && fpv_vehicle_connector_->isApiServerStarted() && getVehicleCount() > 0) {

        if (isRecording() && record_file.is_open()) {
            if (!isLoggingStarted)
            {
                FString imagePathPrefix = common_utils::FileSystem::getLogFileNamePath("img_", "", "", false).c_str();
                FRecordingThread::ThreadInit(imagePathPrefix, this, static_cast<msr::airlib::PhysicsBody*>(fpv_vehicle_connector_->getPhysicsBody()));
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

    for (AActor* actor : spawned_actors_) {
        actor->Destroy();
    }
    if (CameraDirector != nullptr) {
        fpv_vehicle_connector_ = nullptr;
        CameraDirector = nullptr;
    }
    spawned_actors_.Empty();

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

    //find vehicles and cameras available in environment
    //if none available then we will create one
    setupVehiclesAndCamera();

    //detect vehicles in the project and create connector for it
    TArray<AActor*> pawns;
    UAirBlueprintLib::FindAllActor<AFlyingPawn>(this, pawns);
    for (AActor* pawn : pawns) {
        auto vehicle = createVehicle(static_cast<AFlyingPawn*>(pawn));
        if (vehicle != nullptr) {
            vehicles.push_back(vehicle);

            if (pawn == fpv_vehicle_pawn_) {
                fpv_vehicle_connector_ = vehicle;
            }
        }
        //else we don't have vehicle for this pawn
    }
}

ASimModeWorldBase::VehiclePtr ASimModeWorldMultiRotor::createVehicle(AFlyingPawn* pawn)
{
    vehicle_params_ = MultiRotorParamsFactory::createConfig(fpv_vehicle_name);
    auto vehicle = std::make_shared<MultiRotorConnector>();
    vehicle->initialize(pawn, vehicle_params_.get(), enable_rpc, api_server_address, manual_pose_controller);
    return std::static_pointer_cast<VehicleConnectorBase>(vehicle);
}



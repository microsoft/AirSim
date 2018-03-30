#include "SimModeCar.h"
#include "ConstructorHelpers.h"
#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "Car/CarPawn.h"

ASimModeCar::ASimModeCar()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;

    //Try to find the high polycount vehicle
    //If not found, spawn the default class (go-kart)
    static ConstructorHelpers::FClassFinder<ACarPawn> vehicle_pawn_class(TEXT("Blueprint'/AirSim/VehicleAdv/SUV/SuvCarPawn'"));
    if (vehicle_pawn_class.Succeeded()) {
        vehicle_pawn_class_ = vehicle_pawn_class.Class;
        follow_distance_ = -800;
    }
    else {
        vehicle_pawn_class_ = ACarPawn::StaticClass();
        follow_distance_ = -225;
    }
}

void ASimModeCar::BeginPlay()
{
    Super::BeginPlay();

    createVehicles(vehicles_);

    report_wrapper_.initialize(false);
    report_wrapper_.reset();
}

void ASimModeCar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    spawned_actors_.Empty();
    if (CameraDirector != nullptr) {
        fpv_vehicle_pawn_wrapper_ = nullptr;
        CameraDirector = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

VehiclePawnWrapper* ASimModeCar::getFpvVehiclePawnWrapper()
{
    return fpv_vehicle_pawn_wrapper_;
}

void ASimModeCar::setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles)
{
    //get player controller
    APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
    FTransform actor_transform = player_controller->GetViewTarget()->GetActorTransform();
    //put camera little bit above vehicle
    FTransform camera_transform(actor_transform.GetLocation() + FVector(follow_distance_, 0, 400));

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
            CameraDirector->setFollowDistance(follow_distance_);
            CameraDirector->setCameraRotationLagEnabled(true);
            CameraDirector->setFpvCameraIndex(3);
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
        UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);

        //if no vehicle pawns exists in environment
        if (pawns.Num() == 0) {
            //create vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            TVehiclePawn* spawned_pawn = this->GetWorld()->SpawnActor<TVehiclePawn>(
                vehicle_pawn_class_, actor_transform, pawn_spawn_params);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);
        }

        //set up vehicle pawns
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicles.push_back(vehicle_pawn);

            //chose first pawn as FPV if none is designated as FPV
            VehiclePawnWrapper* wrapper = vehicle_pawn->getVehiclePawnWrapper();
            vehicle_pawn->initializeForBeginPlay(getSettings().enable_rpc, 
                getSettings().api_server_address, getSettings().engine_sound);

            if (getSettings().enable_collision_passthrough)
                wrapper->getConfig().enable_passthrough_on_collisions = true;
            if (wrapper->getConfig().is_fpv_vehicle || fpv_vehicle_pawn_wrapper_ == nullptr)
                fpv_vehicle_pawn_wrapper_ = wrapper;
        }
    }

    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_pawn_wrapper_, external_camera);
}


int ASimModeCar::getRemoteControlID(const VehiclePawnWrapper& pawn)
{
    msr::airlib::Settings settings;
    fpv_vehicle_pawn_wrapper_->getRawVehicleSettings(settings);

    msr::airlib::Settings rc_settings;
    settings.getChild("RC", rc_settings);
    return rc_settings.getInt("RemoteControlID", -1);
}

void ASimModeCar::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    //find vehicles and cameras available in environment
    //if none available then we will create one
    setupVehiclesAndCamera(vehicles);
}

void ASimModeCar::reset()
{
    //find all vehicle pawns
    {
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);

        //set up vehicle pawns
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicle_pawn->reset();
        }
    }

    Super::reset();
}

void ASimModeCar::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    report_wrapper_.update();
    report_wrapper_.setEnable(EnableReport);

    if (report_wrapper_.canReport()) {
        report_wrapper_.clearReport();
        updateReport();
    }
}

void ASimModeCar::updateReport()
{
    for (VehiclePtr vehicle : vehicles_) {
        VehiclePawnWrapper* wrapper = vehicle->getVehiclePawnWrapper();
        msr::airlib::StateReporter& reporter = * report_wrapper_.getReporter();
        std::string vehicle_name = fpv_vehicle_pawn_wrapper_->getVehicleConfigName();

        reporter.writeHeading(std::string("Vehicle: ").append(
            vehicle_name == "" ? "(default)" : vehicle_name));

        const msr::airlib::Kinematics::State* kinematics = wrapper->getTrueKinematics();

        reporter.writeValue("Position", kinematics->pose.position);
        reporter.writeValue("Orientation", kinematics->pose.orientation);
        reporter.writeValue("Lin-Vel", kinematics->twist.linear);
        reporter.writeValue("Lin-Accl", kinematics->accelerations.linear);
        reporter.writeValue("Ang-Vel", kinematics->twist.angular);
        reporter.writeValue("Ang-Accl", kinematics->accelerations.angular);
    }
}

std::string ASimModeCar::getReport()
{
    return report_wrapper_.getOutput();
}

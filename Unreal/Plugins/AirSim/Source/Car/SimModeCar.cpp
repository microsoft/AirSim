#include "SimModeCar.h"
#include "ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"

#ifndef AIRLIB_NO_RPC

#pragma warning(disable:4005) //warning C4005: 'TEXT': macro redefinition

#if defined _WIN32 || defined _WIN64
#include "AllowWindowsPlatformTypes.h"
#endif
#include "vehicles/car/api/CarRpcLibServer.hpp"
#if defined _WIN32 || defined _WIN64
#include "HideWindowsPlatformTypes.h"
#endif

#endif

ASimModeCar::ASimModeCar()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;

    follow_distance_ = -800;
}

void ASimModeCar::BeginPlay()
{
    Super::BeginPlay();

    initializePauseState();

    createVehicles(vehicles_);

    debug_reporter_.initialize(false);
    debug_reporter_.reset();
}

void ASimModeCar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    spawned_actors_.Empty();
    if (CameraDirector != nullptr) {
        fpv_vehicle_sim_api_ = nullptr;
        CameraDirector = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

const msr::airlib::VehicleSimApiBase* ASimModeCar::getVehicleSimApi() const
{
    return fpv_vehicle_sim_api_;
}

void ASimModeCar::initializePauseState()
{
    pause_period_ = 0;
    pause_period_start_ = 0;
    pause(false);
}

bool ASimModeCar::isPaused() const
{
    return current_clockspeed_ == 0;
}

void ASimModeCar::pause(bool is_paused)
{
    if (is_paused)
        current_clockspeed_ = 0;
    else
        current_clockspeed_ = getSettings().clock_speed;

    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
}

void ASimModeCar::continueForTime(double seconds)
{
    pause_period_start_ = ClockFactory::get()->nowNanos();
    pause_period_ = seconds;
    pause(false);
}

void ASimModeCar::setupClockSpeed()
{
    current_clockspeed_ = getSettings().clock_speed;

    //setup clock in PhysX
    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
    UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeCar::setupVehiclesAndCamera(std::vector<TVehiclePawn*>& vehicles)
{
    //get player controller
    APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
    FTransform actor_transform = player_controller->GetViewTarget()->GetActorTransform();
    //put camera little bit above vehicle
    FTransform camera_transform(actor_transform.GetLocation() + FVector(follow_distance_, 0, 400));

    //we will either find external camera if it already exist in environment or create one
    APIPCamera* external_camera;


    //find all vehicle pawns
    {
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);

        //add vehicles from settings
        for (auto const& vehicle_setting : getSettings().vehicles)
        {
            if (vehicle_setting.second->auto_create &&
                vehicle_setting.second->vehicle_type == AirSimSettings::kVehicleTypePhysXCar) {

                //decide which derived BP to use
                std::string pawn_path = vehicle_setting.second->pawn_path;
                if (pawn_path == "")
                    pawn_path = "DefaultCar";

                //create vehicle pawn
                FActorSpawnParameters pawn_spawn_params;
                pawn_spawn_params.Name = FName(vehicle_setting.second->vehicle_name.c_str());
                pawn_spawn_params.SpawnCollisionHandlingOverride =
                    ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

                auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                    getSettings().pawn_paths.at(pawn_path).pawn_bp);

                TVehiclePawn* spawned_pawn = this->GetWorld()->SpawnActor<TVehiclePawn>(
                    vehicle_bp_class, actor_transform, pawn_spawn_params);

                spawned_actors_.Add(spawned_pawn);
                pawns.Add(spawned_pawn);
            }
        }

        //set up vehicle pawns
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicles.push_back(vehicle_pawn);

            //chose first pawn as FPV if none is designated as FPV
            VehicleSimApi* vehicle_sim_api = vehicle_pawn->getVehicleSimApi();
            vehicle_sim_api->initialize(this, cameras, std::string(TCHAR_TO_UTF8(*this->GetName())));
            vehicle_sim_api->setKinematics(&kinematics_);

            vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);

            if (getSettings().enable_collision_passthrough)
                vehicle_sim_api->getConfig().enable_passthrough_on_collisions = true;
            if (vehicle_sim_api->getConfig().is_fpv_vehicle || fpv_vehicle_sim_api_ == nullptr)
                fpv_vehicle_sim_api_ = vehicle_sim_api;
        }
    }

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

    fpv_vehicle_sim_api_->possess();
    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_sim_api_, external_camera);
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeCar::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
        getApiProvider(), getSettings().api_server_address));
#endif
}

int ASimModeCar::getRemoteControlID(const VehicleSimApi& pawn) const
{
    return fpv_vehicle_sim_api_->getRemoteControlID();
}

void ASimModeCar::createVehicles(std::vector<TVehiclePawn*>& vehicles)
{
    //find vehicles and cameras available in environment
    //if none available then we will create one
    setupVehiclesAndCamera(vehicles);
}

void ASimModeCar::reset()
{
    msr::airlib::VehicleApiBase* api = getVehicleApi();
    if (api) {
        UAirBlueprintLib::RunCommandOnGameThread([api]() {
            api->reset();
        }, true);
    }

    Super::reset();
}

void ASimModeCar::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    if (pause_period_start_ > 0) {
        if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
            if (!isPaused())
                pause(true);

            pause_period_start_ = 0;
        }
    }

    debug_reporter_.update();
    debug_reporter_.setEnable(EnableReport);

    if (debug_reporter_.canReport()) {
        debug_reporter_.clearReport();
        updateDebugReport();
    }
}

void ASimModeCar::updateDebugReport()
{
    for (TVehiclePawn* vehicle : vehicles_) {
        VehicleSimApi* vehicle_sim_api = vehicle->getVehicleSimApi();
        msr::airlib::StateReporter& reporter = *debug_reporter_.getReporter();
        std::string vehicle_name = fpv_vehicle_sim_api_->getVehicleSetting()->vehicle_name;

        reporter.writeHeading(std::string("Vehicle: ").append(
            vehicle_name == "" ? "(default)" : vehicle_name));

        const msr::airlib::Kinematics::State* kinematics = vehicle_sim_api->getGroundTruthKinematics();

        reporter.writeValue("Position", kinematics->pose.position);
        reporter.writeValue("Orientation", kinematics->pose.orientation);
        reporter.writeValue("Lin-Vel", kinematics->twist.linear);
        reporter.writeValue("Lin-Accl", kinematics->accelerations.linear);
        reporter.writeValue("Ang-Vel", kinematics->twist.angular);
        reporter.writeValue("Ang-Accl", kinematics->accelerations.angular);
    }
}

std::string ASimModeCar::getDebugReport()
{
    return debug_reporter_.getOutput();
}

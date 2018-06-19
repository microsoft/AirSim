#include "SimModeCar.h"
#include "ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"

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
    //TODO: get this from settings
    follow_distance_ = -800;
}

void ASimModeCar::BeginPlay()
{
    Super::BeginPlay();

    setupVehiclesAndCamera();
    for (auto* api : getApiProvider()->getVehicleSimApis()) {
        api->reset();
    }
    checkVehicleReady();

    initializePauseState();
}

void ASimModeCar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    spawned_actors_.Empty();
    vehicle_sim_apis_.clear();
    Super::EndPlay(EndPlayReason);
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

void ASimModeCar::setupVehiclesAndCamera()
{
    //get UU origin of global NED frame
    FTransform uu_origin = getGlobalNedTransform().getGlobalTransform();
    //TODO:make this configurable
    FTransform camera_transform(FRotator::ZeroRotator, uu_origin.GetLocation() + FVector(follow_distance_, 0, 400));
    initializeCameraDirector(camera_transform, follow_distance_);

    //find all vehicle pawns
    {
        TArray<AActor*> pawns;
        UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);

        TVehiclePawn* fpv_pawn = nullptr;

        //add vehicles from settings
        for (auto const& vehicle_setting_pair : getSettings().vehicles)
        {
            //if vehicle is of multirotor type and auto creatable
            const auto& vehicle_setting = *vehicle_setting_pair.second;
            if (vehicle_setting.auto_create &&
                ((vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypePhysXCar) )) {

                //decide which derived BP to use
                std::string pawn_path = vehicle_setting.pawn_path;
                if (pawn_path == "")
                    pawn_path = "DefaultCar";

                //compute initial pose
                FVector spawn_position = uu_origin.GetLocation();
                FRotator spawn_rotation = uu_origin.Rotator();
                Vector3r settings_position = vehicle_setting.position;
                if (!VectorMath::hasNan(settings_position))
                    spawn_position = getGlobalNedTransform().fromLocalNed(settings_position);
                const auto& rotation = vehicle_setting.rotation;
                if (!std::isnan(rotation.yaw))
                    spawn_rotation.Yaw = rotation.yaw;
                if (!std::isnan(rotation.pitch))
                    spawn_rotation.Pitch = rotation.pitch;
                if (!std::isnan(rotation.roll))
                    spawn_rotation.Roll = rotation.roll;

                //spawn vehicle pawn
                FActorSpawnParameters pawn_spawn_params;
                pawn_spawn_params.Name = FName(vehicle_setting.vehicle_name.c_str());
                pawn_spawn_params.SpawnCollisionHandlingOverride =
                    ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                    getSettings().pawn_paths.at(pawn_path).pawn_bp);
                TVehiclePawn* spawned_pawn = this->GetWorld()->SpawnActor<TVehiclePawn>(
                    vehicle_bp_class, FTransform(spawn_rotation, spawn_position), pawn_spawn_params);

                spawned_actors_.Add(spawned_pawn);
                pawns.Add(spawned_pawn);

                if (vehicle_setting.is_fpv_vehicle)
                    fpv_pawn = spawned_pawn;
            }
        }

        //create API objects for each pawn we have
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);

            //create vehicle sim api
            const auto& ned_transform = getGlobalNedTransform();
            const auto& pawn_ned_pos = ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
            const auto& home_geopoint= msr::airlib::EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);
            auto vehicle_sim_api = std::unique_ptr<CarPawnSimApi>(new CarPawnSimApi(
                vehicle_pawn, getGlobalNedTransform(),
                vehicle_pawn->getPawnEvents(), vehicle_pawn->getCameras(), pip_camera_class, collision_display_template,
                vehicle_pawn->getKeyBoardControls(), vehicle_pawn->getVehicleMovementComponent(), home_geopoint));

            std::string vehicle_name = vehicle_sim_api->getVehicleName();

            auto vehicle_api = vehicle_sim_api->getVehicleApi();
            auto vehicle_sim_api_p = vehicle_sim_api.get();
            getApiProvider()->insert_or_assign(vehicle_name, vehicle_api, vehicle_sim_api_p);
            if ((fpv_pawn == vehicle_pawn || !getApiProvider()->hasDefaultVehicle()) && vehicle_name != "")
                getApiProvider()->makeDefaultVehicle(vehicle_name);

            vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
        }
    }
    
    if (getApiProvider()->hasDefaultVehicle()) {
        //TODO: better handle no FPV vehicles scenario
        getVehicleSimApi()->possess();
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), getVehicleSimApi()->getPawn(),
            getVehicleSimApi()->getCamera("fpv"), getVehicleSimApi()->getCamera(""), getVehicleSimApi()->getCamera("back_center"));
    }
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
}



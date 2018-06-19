#include "SimModeComputerVision.h"
#include "ConstructorHelpers.h"
#include "Engine/World.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "PawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "physics/Kinematics.hpp"


#ifndef AIRLIB_NO_RPC

#pragma warning(disable:4005) //warning C4005: 'TEXT': macro redefinition

#if defined _WIN32 || defined _WIN64
#include "AllowWindowsPlatformTypes.h"
#endif
#include "api/RpcLibServerBase.hpp"
#if defined _WIN32 || defined _WIN64
#include "HideWindowsPlatformTypes.h"
#endif

#endif

ASimModeComputerVision::ASimModeComputerVision()
{
    //TODO: get this from settings
    follow_distance_ = -200;    
}

void ASimModeComputerVision::BeginPlay()
{
    Super::BeginPlay();
    
    setupVehiclesAndCamera();
    for (auto* api : getApiProvider()->getVehicleSimApis()) {
        api->reset();
    }
    checkVehicleReady();
}

void ASimModeComputerVision::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    spawned_actors_.Empty();
    vehicle_sim_apis_.clear();

    Super::EndPlay(EndPlayReason);
}

void ASimModeComputerVision::pause(bool is_paused)
{
    unused(is_paused);
    //no effect in this sim mode
}

void ASimModeComputerVision::continueForTime(double seconds)
{
    unused(seconds);
    //no effect in this sim mode
}

void ASimModeComputerVision::setupVehiclesAndCamera()
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
                ((vehicle_setting.vehicle_type == msr::airlib::AirSimSettings::kVehicleTypeComputerVision) )) {

                //decide which derived BP to use
                std::string pawn_path = vehicle_setting.pawn_path;
                if (pawn_path == "")
                    pawn_path = "DefaultComputerVision";

                //compute initial pose
                FVector spawn_position = uu_origin.GetLocation();
                FRotator spawn_rotation = uu_origin.Rotator();
                msr::airlib::Vector3r settings_position = vehicle_setting.position;
                if (!msr::airlib::VectorMath::hasNan(settings_position))
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
            vehicle_pawn->initializeForBeginPlay();

            //create vehicle sim api
            const auto& ned_transform = getGlobalNedTransform();
            const auto& pawn_ned_pos = ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
            const auto& home_geopoint= msr::airlib::EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);
            auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new PawnSimApi(
                vehicle_pawn, getGlobalNedTransform(),
                vehicle_pawn->getPawnEvents(), vehicle_pawn->getCameras(), pip_camera_class, collision_display_template, home_geopoint));

            std::string vehicle_name = vehicle_sim_api->getVehicleName();

            auto vehicle_sim_api_p = vehicle_sim_api.get();
            getApiProvider()->insert_or_assign(vehicle_name, static_cast<msr::airlib::VehicleApiBase*>(nullptr), vehicle_sim_api_p);
            if ((fpv_pawn == vehicle_pawn || !getApiProvider()->hasDefaultVehicle()) && vehicle_name != "")
                getApiProvider()->makeDefaultVehicle(vehicle_name);

            vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
        }
    }
    
    if (getApiProvider()->hasDefaultVehicle()) {
        //TODO: better handle no FPV vehicles scenario
        getVehicleSimApi()->possess();
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), getVehicleSimApi()->getPawn(),
            getVehicleSimApi()->getCamera("fpv"), getVehicleSimApi()->getCamera(""), nullptr);
    }
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeComputerVision::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::RpcLibServerBase(
        getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeComputerVision::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}



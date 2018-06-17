#include "SimModeWorldMultiRotor.h"
#include "ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "MultirotorPawnSimApi.h"
#include "physics/PhysicsBody.hpp"
#include "common/ClockFactory.hpp"
#include <memory>


#ifndef AIRLIB_NO_RPC

#pragma warning(disable:4005) //warning C4005: 'TEXT': macro redefinition

#if defined _WIN32 || defined _WIN64
#include "AllowWindowsPlatformTypes.h"
#endif
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#if defined _WIN32 || defined _WIN64
#include "HideWindowsPlatformTypes.h"
#endif

#endif


ASimModeWorldMultiRotor::ASimModeWorldMultiRotor()
{
    //TODO: get this from settings
    follow_distance_ = -225;
}

void ASimModeWorldMultiRotor::BeginPlay()
{
    Super::BeginPlay();

    setupVehiclesAndCamera();

    //let base class setup physics world
    initializeForPlay();

    checkVehicleReady();
}

void ASimModeWorldMultiRotor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismantle
    stopAsyncUpdator();

    vehicle_sim_apis_.clear();
    spawned_actors_.Empty();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldMultiRotor::setupVehiclesAndCamera()
{
    //get UU origin of global NED frame
    FVector uu_origin = getGlobalNedTransform().getGlobalOffset();

    //TODO:make this configurable
    FTransform camera_transform(uu_origin + FVector(-300, 0, 200));
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
                ((vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) ||
                 (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypePX4))) {

                //decide which derived BP to use
                std::string pawn_path = vehicle_setting.pawn_path;
                if (pawn_path == "")
                    pawn_path = "DefaultQuadrotor";

                //compute initial pose
                FVector spawn_position = uu_origin;
                FRotator spawn_rotation = FRotator::ZeroRotator;
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
            //initialize the pawn
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicle_pawn->initializeForBeginPlay();

            //create vehicle sim api
            const auto& ned_transform = getGlobalNedTransform();
            const auto& pawn_ned_pos = ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
            const auto& home_geopoint= EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);
            auto vehicle_sim_api = std::unique_ptr<MultirotorPawnSimApi>(new MultirotorPawnSimApi(
                vehicle_pawn, ned_transform,
                vehicle_pawn->getPawnEvents(), vehicle_pawn->getCameras(), pip_camera_class, collision_display_template,
                home_geopoint));

            std::string vehicle_name = vehicle_sim_api->getVehicleName();

            getApiProvider()->insert_or_assign(vehicle_name, vehicle_sim_api->getVehicleApi(),
                vehicle_sim_api.get());
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
    else
        CameraDirector->initializeForBeginPlay(getInitialViewMode(), nullptr, nullptr, nullptr, nullptr);
}

void ASimModeWorldMultiRotor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeWorldMultiRotor::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(
        getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeWorldMultiRotor::setupClockSpeed()
{
    typedef msr::airlib::ClockFactory ClockFactory;

    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock") {
        //scalable clock returns interval same as wall clock but multiplied by a scale factor
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    }
    else if (clock_type == "SteppableClock") {
        //steppable clock returns interval that is a constant number irrespective of wall clock
        //we can either multiply this fixed interval by scale factor to speed up/down the clock
        //but that would cause vehicles like quadrotors to become unstable
        //so alternative we use here is instead to scale control loop frequency. The downside is that
        //depending on compute power available, we will max out control loop frequency and therefore can no longer
        //get increase in clock speed

        //Approach 1: scale clock period, no longer used now due to quadrotor instability
        //ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
        //static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

        //Approach 2: scale control loop frequency if clock is speeded up
        if (clock_speed >= 1) {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9))); //no clock_speed multiplier

            setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
        }
        else {
            //for slowing down, this don't generate instability
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
        }
    }
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}


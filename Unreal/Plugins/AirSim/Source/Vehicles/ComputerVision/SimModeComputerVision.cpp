#include "SimModeComputerVision.h"
#include "ConstructorHelpers.h"
#include "Engine/World.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "PawnSimAPi.h"
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

    debug_reporter_.initialize(false);
    debug_reporter_.reset();
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
                TVehiclePawn* spawned_pawn = this->GetWorld()->SpawnActor<TVehiclePawn>(
                    spawn_position, spawn_rotation, pawn_spawn_params);

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
    
    for (auto* api : getApiProvider()->getVehicleSimApis()) {
        api->updateRenderedState(DeltaSeconds);
        api->updateRendering(DeltaSeconds);
    }

    debug_reporter_.update();
    debug_reporter_.setEnable(EnableReport);

    if (debug_reporter_.canReport()) {
        debug_reporter_.clearReport();
        updateDebugReport();
    }
}

void ASimModeComputerVision::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        for (auto& api : getApiProvider()->getVehicleSimApis()) {
            api->reset();
        }
    }, true);

    Super::reset();
}

void ASimModeComputerVision::updateDebugReport()
{
    for (auto& api : getApiProvider()->getVehicleSimApis()) {
        PawnSimApi* vehicle_sim_api = static_cast<PawnSimApi*>(api);
        msr::airlib::StateReporter& reporter = *debug_reporter_.getReporter();
        std::string vehicle_name = vehicle_sim_api->getVehicleName();

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

std::string ASimModeComputerVision::getDebugReport()
{
    return debug_reporter_.getOutput();
}

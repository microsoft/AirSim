#include "SimModeCar.h"
#include "ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "CarPawnSimAPi.h"
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
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;

    //TODO: get this from settings
    follow_distance_ = -800;
}

void ASimModeCar::BeginPlay()
{
    Super::BeginPlay();

    initializePauseState();

    setupVehiclesAndCamera();

    debug_reporter_.initialize(false);
    debug_reporter_.reset();
}

void ASimModeCar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    spawned_actors_.Empty();
    
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

UClass* ASimModeCar::getExternalCameraClass()
{
    return external_camera_class_;
}

void ASimModeCar::setupVehiclesAndCamera()
{
    //get UU origin of global NED frame
    FVector uu_origin = getGlobalNedTransform().getLocalOffset();
    //TODO:make this configurable
    FTransform camera_transform(uu_origin + FVector(follow_distance_, 0, 400));
    initializeCameraDirector(camera_transform);

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
            //initialize each vehicle pawn we found
            TVehiclePawn* vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
            vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);

            //create vehicle sim api
            const auto& ned_transform = getGlobalNedTransform();
            const auto& pawn_ned_pos = ned_transform.toLocalNed(vehicle_pawn->GetActorLocation());
            const auto& home_geopoint= msr::airlib::EarthUtils::nedToGeodetic(pawn_ned_pos, getSettings().origin_geopoint);
            auto vehicle_sim_api = std::unique_ptr<CarPawnSimApi>(new CarPawnSimApi(
                vehicle_pawn, getGlobalNedTransform(),
                vehicle_pawn->getCollisionSignal(), vehicle_pawn->getCameras(), vehicle_pawn->getKeyBoardControls(),
                vehicle_pawn->getVehicleMovementComponent(), home_geopoint));

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
    
    for (auto& pair : getApiProvider()->getVehicleSimApis()) {
        pair.second->updateRenderedState(DeltaSeconds);
        pair.second->updateRendering(DeltaSeconds);
    }
    
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

void ASimModeCar::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        for (auto& pair : getApiProvider()->getVehicleSimApis()) {
            if (pair.second)
                pair.second->reset();
        }
    }, true);

    Super::reset();
}

void ASimModeCar::updateDebugReport()
{
    for (auto& pair : getApiProvider()->getVehicleSimApis()) {
        PawnSimApi* vehicle_sim_api = static_cast<PawnSimApi*>(pair.second);
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

std::string ASimModeCar::getDebugReport()
{
    return debug_reporter_.getOutput();
}

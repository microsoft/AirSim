#include "SimModeWorldMultiRotor.h"
#include "ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/ClockFactory.hpp"
#include <memory>
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"

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
    static ConstructorHelpers::FClassFinder<APIPCamera> external_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    external_camera_class_ = external_camera_class.Succeeded() ? external_camera_class.Class : nullptr;
    static ConstructorHelpers::FClassFinder<ACameraDirector> camera_director_class(TEXT("Blueprint'/AirSim/Blueprints/BP_CameraDirector'"));
    camera_director_class_ = camera_director_class.Succeeded() ? camera_director_class.Class : nullptr;
}

void ASimModeWorldMultiRotor::BeginPlay()
{
    Super::BeginPlay();
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

void ASimModeWorldMultiRotor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismantle
    stopAsyncUpdator();

    //for (AActor* actor : spawned_actors_) {
    //    actor->Destroy();
    //}
    spawned_actors_.Empty();
    //fpv_vehicle_connectors_.Empty();
    CameraDirector = nullptr;

    Super::EndPlay(EndPlayReason);
}

VehicleSimApi* ASimModeWorldMultiRotor::getFpvVehicleSimApi() const
{
    return fpv_vehicle_sim_api_;
}

void ASimModeWorldMultiRotor::setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles)
{
    //put camera little bit above vehicle
    //TODO: allow setting camera pose from settings
    FVector uu_origin = getGlobalNedTransform().getLocalOffset();
    FTransform camera_transform(uu_origin + FVector(-300, 0, 200));

    //we will either find external camera if it already exist in environment or create one
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
            CameraDirector->setFollowDistance(225);
            CameraDirector->setCameraRotationLagEnabled(false);
            CameraDirector->setFpvCameraIndex(0);
            CameraDirector->enableFlyWithMeMode();
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

        //add vehicles from settings
        for (auto const& vehicle_setting_pair : getSettings().vehicles)
        {
            const auto& vehicle_setting = *vehicle_setting_pair.second;
            if (vehicle_setting.auto_create &&
                ((vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) ||
                 (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypePX4))) {

                //decide which derived BP to use
                std::string pawn_path = vehicle_setting.pawn_path;
                if (pawn_path == "")
                    pawn_path = "DefaultQuadrotor";

                //create vehicle pawn
                FActorSpawnParameters pawn_spawn_params;
                pawn_spawn_params.Name = FName(vehicle_setting.vehicle_name.c_str());
                pawn_spawn_params.SpawnCollisionHandlingOverride =
                    ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

                auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                    getSettings().pawn_paths.at("DefaultQuadrotor").pawn_bp);

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
                TMultiRotorPawn* spawned_pawn = this->GetWorld()->SpawnActor<TMultiRotorPawn>(
                    vehicle_bp_class, FTransform(spawn_rotation, spawn_position), pawn_spawn_params);

                spawned_actors_.Add(spawned_pawn);
                pawns.Add(spawned_pawn);
            }
        }

        //set up vehicle pawns
        fpv_vehicle_sim_api_ = nullptr;
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TMultiRotorPawn* vehicle_pawn = static_cast<TMultiRotorPawn*>(pawn);
            vehicle_pawn->initializeForBeginPlay(getGlobalNedTransform(), std::string(TCHAR_TO_UTF8(*this->GetName())));
            const auto* vehicle_sim_api = vehicle_pawn->getVehicleSimApi();
            //chose first pawn as FPV if none is designated as FPV
            if (vehicle_sim_api->getVehicleSetting().is_fpv_vehicle || fpv_vehicle_sim_api_ == nullptr)
                fpv_vehicle_sim_api_ = vehicle_sim_api;

            //now create the connector for each pawn
            VehiclePtr vehicle = createVehicle(vehicle_sim_api);
            if (vehicle != nullptr) {
                vehicles.push_back(vehicle);
                fpv_vehicle_connectors_.Add(vehicle);
            }
            //else we don't have vehicle for this pawn
        }
    }

    fpv_vehicle_sim_api_->possess();
    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_sim_api_, external_camera);
}


void ASimModeWorldMultiRotor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    getFpvVehicleSimApi()->setLogLine(getLogString());
}

std::string ASimModeWorldMultiRotor::getLogString() const
{
    const msr::airlib::Kinematics::State* kinematics = getFpvVehicleSimApi()->getGroundTruthKinematics();
    uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

    //TODO: because this bug we are using alternative code with stringstream
    //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html

    std::string line;
    line.append(std::to_string(timestamp_millis)).append("\t")
        .append(std::to_string(kinematics->pose.position.x())).append("\t")
        .append(std::to_string(kinematics->pose.position.y())).append("\t")
        .append(std::to_string(kinematics->pose.position.z())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.w())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.x())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.y())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.z())).append("\t");

    return line;

    //std::stringstream ss;
    //ss << timestamp_millis << "\t";
    //ss << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
    //ss << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
    //ss << "\n";
    //return ss.str();
}

void ASimModeWorldMultiRotor::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    //find vehicles and cameras available in environment
    //if none available then we will create one
    setupVehiclesAndCamera(vehicles);
}

ASimModeWorldBase::VehiclePtr ASimModeWorldMultiRotor::createVehicle(VehicleSimApi* vehicle_sim_api)
{
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(vehicle_sim_api->getPawn(), &vehicle_sim_api->getNedTransform());
    auto vehicle_params = MultiRotorParamsFactory::createConfig(vehicle_sim_api->getVehicleSetting(), sensor_factory);

    vehicle_params_.push_back(std::move(vehicle_params));

    std::shared_ptr<MultirotorSimApi> vehicle = std::make_shared<MultirotorSimApi>(
        vehicle_sim_api, vehicle_params_.back().get(), manual_pose_controller);

    if (vehicle->getPhysicsBody() != nullptr)
        vehicle_sim_api->setKinematics(&(static_cast<PhysicsBody*>(vehicle->getPhysicsBody())->getKinematics()));

    return std::static_pointer_cast<VehicleSimApiBase>(vehicle);
}

void ASimModeWorldMultiRotor::setupClockSpeed()
{
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


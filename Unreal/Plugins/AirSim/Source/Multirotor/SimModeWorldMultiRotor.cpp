#include "SimModeWorldMultiRotor.h"
#include "ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
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
        getSimModeApi(), getSettings().api_server_address));
#endif
}

void ASimModeWorldMultiRotor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismental
    stopAsyncUpdator();

    //for (AActor* actor : spawned_actors_) {
    //    actor->Destroy();
    //}
    spawned_actors_.Empty();
    //fpv_vehicle_connectors_.Empty();
    CameraDirector = nullptr;

    Super::EndPlay(EndPlayReason);
}

VehiclePawnWrapper* ASimModeWorldMultiRotor::getFpvVehiclePawnWrapper() const
{
    return fpv_vehicle_pawn_wrapper_;
}

std::vector<std::string> split(const std::string &string, const char separator, const bool keepEmptyParts = false)
{
    std::istringstream ss(string);
    std::vector<std::string> stringList;
    while (!ss.eof())
    {
        std::string str;
        std::getline(ss, str, separator);
        if ((!keepEmptyParts) && str.empty())
        {
            continue;
        }

        stringList.push_back(str);
    }

    return stringList;
}

std::string w2a(const std::wstring &wstring)
{
    std::string string(wstring.size(), 0);
    for (size_t i = 0; i < wstring.size(); ++i)
    {
        string[i] = wstring[i];  // TODO only support ASCII strings
    }

    return string;
}

void ASimModeWorldMultiRotor::setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles)
{
    //get player controller
    APlayerController* player_controller = this->GetWorld()->GetFirstPlayerController();
    FTransform actor_transform = player_controller->GetViewTarget()->GetActorTransform();
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
        TArray<AActor*> unrealProjectPawns;
        UAirBlueprintLib::FindAllActor<TMultiRotorPawn>(this, unrealProjectPawns);
        for (AActor* pawn : unrealProjectPawns)
        {
            TMultiRotorPawn* vehicle_pawn = static_cast<TMultiRotorPawn*>(pawn);
            if (!vehicle_pawn)
            {
                continue;
            }

            vehicle_pawn->initializeForBeginPlay(getSettings().additional_camera_settings);
            VehiclePawnWrapper* wrapper = vehicle_pawn->getVehiclePawnWrapper();
            wrapper->getConfig().vehicle_config_name = w2a(std::wstring(*pawn->GetName()));
        }

        TArray<AActor*> additionalPawns;
        const std::string additionalVehicleConfigs_ = Settings::singleton().getString("AdditionalVehicleConfigs", "");
        const std::vector<std::string> additionalVehicleConfigs = split(additionalVehicleConfigs_, ';');
        if (!additionalVehicleConfigs.empty())
        {
            std::vector<std::vector<double> > additionalVehiclePositions(additionalVehicleConfigs.size());
            for (size_t i = 0; i < additionalVehiclePositions.size(); ++i)
            {
                std::vector<double> &additionalVehiclePosition = additionalVehiclePositions[i];
                additionalVehiclePosition.resize(3);
                additionalVehiclePosition[0] = 5 * i;
                additionalVehiclePosition[1] = 0;
                additionalVehiclePosition[2] = 0;
            }
            const std::string additionalVehiclePositions__ = Settings::singleton().getString("AdditionalVehiclePositions", "");
            const std::vector<std::string> additionalVehiclePositions_ = split(additionalVehiclePositions__, ';');
            for (size_t i = 0; i < std::min(additionalVehiclePositions.size(), additionalVehiclePositions_.size()); ++i)
            {
                const std::string &additionalVehiclePosition__ = additionalVehiclePositions_[i];
                const std::vector<std::string> additionalVehiclePosition_ = split(additionalVehiclePosition__, ',');
                std::vector<double> &additionalVehiclePosition = additionalVehiclePositions[i];
                for (size_t j = 0; j < std::min(additionalVehiclePosition.size(), additionalVehiclePosition_.size()); ++j)
                {
                    additionalVehiclePosition[j] = atof(additionalVehiclePosition_[j].c_str());
                }
            }

            auto vehicle_bp_class = UAirBlueprintLib::LoadClass(getSettings().pawn_paths.at("DefaultQuadrotor").pawn_bp);

            for (size_t i = 0; i < additionalVehicleConfigs.size(); ++i)
            {
                const std::string &additionalVehicleConfig = additionalVehicleConfigs[i];
                const std::vector<double> &additionalVehiclePosition = additionalVehiclePositions[i];

                //create vehicle pawn
                FActorSpawnParameters pawn_spawn_params;
                pawn_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
                FTransform actor_transform_ = actor_transform;
                actor_transform_.AddToTranslation(FVector(100 * additionalVehiclePosition[0], 100 * additionalVehiclePosition[1], 100 * additionalVehiclePosition[2]));  // 100: cm -> m
                TMultiRotorPawn* spawned_pawn = this->GetWorld()->SpawnActor<TMultiRotorPawn>(vehicle_bp_class, actor_transform_, pawn_spawn_params);

                spawned_pawn->initializeForBeginPlay(getSettings().additional_camera_settings);
                VehiclePawnWrapper* wrapper = spawned_pawn->getVehiclePawnWrapper();
                wrapper->getConfig().vehicle_config_name = additionalVehicleConfig;

                spawned_actors_.Add(spawned_pawn);
                additionalPawns.Add(spawned_pawn);
            }
        }

        TArray<AActor*> pawns;
        pawns.Append(unrealProjectPawns);
        pawns.Append(additionalPawns);

        const std::string fpvVehicleConfig = Settings::singleton().getString("FPVVehicleConfig", "");
        if (!fpvVehicleConfig.empty())
        {
            for (AActor* pawn : pawns)
            {
                TMultiRotorPawn* vehicle_pawn = static_cast<TMultiRotorPawn*>(pawn);
                VehiclePawnWrapper* wrapper = vehicle_pawn->getVehiclePawnWrapper();

                if (wrapper->getConfig().vehicle_config_name == fpvVehicleConfig)
                {
                    wrapper->getConfig().is_fpv_vehicle = true;
                    break;
                }
            }
        }

        //if no vehicle pawns exists in environment
        if (pawns.Num() == 0) {
            auto vehicle_bp_class = UAirBlueprintLib::LoadClass(
                getSettings().pawn_paths.at("DefaultQuadrotor").pawn_bp);

            //create vehicle pawn
            FActorSpawnParameters pawn_spawn_params;
            pawn_spawn_params.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
            TMultiRotorPawn* spawned_pawn = this->GetWorld()->SpawnActor<TMultiRotorPawn>(
                vehicle_bp_class, actor_transform, pawn_spawn_params);

            spawned_pawn->initializeForBeginPlay(getSettings().additional_camera_settings);

            spawned_actors_.Add(spawned_pawn);
            pawns.Add(spawned_pawn);
        }

        //set up vehicle pawns
        for (AActor* pawn : pawns)
        {
            //initialize each vehicle pawn we found
            TMultiRotorPawn* vehicle_pawn = static_cast<TMultiRotorPawn*>(pawn);
//            vehicle_pawn->initializeForBeginPlay(getSettings().additional_camera_settings);

            //chose first pawn as FPV if none is designated as FPV
            VehiclePawnWrapper* wrapper = vehicle_pawn->getVehiclePawnWrapper();

            if (getSettings().enable_collision_passthrough)
                wrapper->getConfig().enable_passthrough_on_collisions = true;
            if (wrapper->getConfig().is_fpv_vehicle || fpv_vehicle_pawn_wrapper_ == nullptr)
                fpv_vehicle_pawn_wrapper_ = wrapper;

            //now create the connector for each pawn
            VehiclePtr vehicle = createVehicle(wrapper);
            if (vehicle != nullptr) {
                vehicles.push_back(vehicle);
                fpv_vehicle_connectors_.Add(vehicle);
            }
            //else we don't have vehicle for this pawn
        }
    }

    fpv_vehicle_pawn_wrapper_->possess();
    CameraDirector->initializeForBeginPlay(getInitialViewMode(), fpv_vehicle_pawn_wrapper_, external_camera);
}


void ASimModeWorldMultiRotor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    getFpvVehiclePawnWrapper()->setLogLine(getLogString());
}

std::string ASimModeWorldMultiRotor::getLogString() const
{
    const msr::airlib::Kinematics::State* kinematics = getFpvVehiclePawnWrapper()->getTrueKinematics();
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

class SimModeApi_ : public SimModeApiBase
{
public:
    SimModeApi_(ASimModeBase* simMode, VehiclePawnWrapper *vehiclePawnWrapper)
        : mSimMode(simMode)
        , mVehiclePawnWrapper(vehiclePawnWrapper)
    {
    }
    virtual VehicleApiBase* getVehicleApi() override
    {
        return mVehiclePawnWrapper ? mVehiclePawnWrapper->getApi() : mSimMode->getVehicleApi();
    }
    virtual void reset() override
    {
        mSimMode->reset();
    }
    virtual bool isPaused() const override
    {
        return mSimMode->isPaused();
    }
    virtual void pause(bool is_paused) override
    {
        mSimMode->pause(is_paused);
    }
    virtual void continueForTime(double seconds) override
    {
        mSimMode->continueForTime(seconds);
    }

private:
    ASimModeBase *mSimMode;
    VehiclePawnWrapper *mVehiclePawnWrapper;
};

ASimModeWorldBase::VehiclePtr ASimModeWorldMultiRotor::createVehicle(VehiclePawnWrapper* wrapper)
{
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(wrapper->getPawn(), &wrapper->getNedTransform());
    auto vehicle_params = MultiRotorParamsFactory::createConfig(wrapper->getVehicleConfigName(), sensor_factory);

    vehicle_params_.push_back(std::move(vehicle_params));

    std::shared_ptr<MultiRotorConnector> vehicle = std::make_shared<MultiRotorConnector>(
        wrapper, vehicle_params_.back().get(), manual_pose_controller);

    if (vehicle->getPhysicsBody() != nullptr)
        wrapper->setKinematics(&(static_cast<PhysicsBody*>(vehicle->getPhysicsBody())->getKinematics()));

    AirSimSettings::VehicleSettings vehicle_settings = AirSimSettings::singleton().getVehicleSettings(wrapper->getVehicleConfigName());
    static std::vector<std::unique_ptr<ApiServerBase> > apiServers;
#ifdef AIRLIB_NO_RPC
    apiServers.push_back(ASimModeBase::createApiServer());
#else
    apiServers.push_back(std::unique_ptr<ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(new SimModeApi_(this, wrapper), getSettings().api_server_address, vehicle_settings.server_port)));
#endif
    try
    {
        apiServers.back()->start();
        UAirBlueprintLib::LogMessageString("Started RpcLib Server at " + getSettings().api_server_address + ":" + std::to_string(vehicle_settings.server_port), "", LogDebugLevel::Informational);
    }
    catch (std::exception& ex)
    {
        UAirBlueprintLib::LogMessageString("Cannot start RpcLib Server", ex.what(), LogDebugLevel::Failure);
    }

    return std::static_pointer_cast<VehicleConnectorBase>(vehicle);
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

        //Approach 1: scale clock period, no longer used now due to quadrotor unstability
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


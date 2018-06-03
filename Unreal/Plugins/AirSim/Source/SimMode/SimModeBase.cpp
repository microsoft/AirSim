#include "SimModeBase.h"
#include "Recording/RecordingThread.h"
#include "Misc/MessageDialog.h"
#include "Misc/EngineVersion.h"
#include "Runtime/Launch/Resources/Version.h"
#include "ConstructorHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/OutputDeviceNull.h"


#include <memory>
#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "common/ScalableClock.hpp"
#include "common/SteppableClock.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include "common/EarthCelestial.hpp"

//TODO: this is going to cause circular references which is fine here but
//in future we should consider moving SimMode not derived from AActor and move
//it to AirLib and directly implement WorldSimApiBase interface
#include "WorldSimApi.h"


const char ASimModeBase::kUsageScenarioComputerVision[] = "ComputerVision";


ASimModeBase::ASimModeBase()
{
    PrimaryActorTick.bCanEverTick = true;

    static ConstructorHelpers::FClassFinder<AActor> sky_sphere_class(TEXT("Blueprint'/Engine/EngineSky/BP_Sky_Sphere'"));
    sky_sphere_class_ = sky_sphere_class.Succeeded() ? sky_sphere_class.Class : nullptr;
    
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    global_ned_transform_ = NedTransform(GetActorLocation(), 
        UAirBlueprintLib::GetWorldToMetersScale(this));
    world_sim_api_.reset(new WorldSimApi(this));

    setupPhysicsLoopPeriod();

    setupClockSpeed();

    setStencilIDs();
    
    record_tick_count = 0;
    setupInputBindings();

    setupTimeOfDay();

    checkVehicleReady();

    UAirBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""), LogDebugLevel::Informational);
}

const NedTransform& ASimModeBase::getGlobalNedTransform()
{
    return global_ned_transform_;
}

void ASimModeBase::checkVehicleReady()
{
    const auto& vehicle_apis = api_provider_->getVehicleApis();
    for (auto it = vehicle_apis.begin(); it != vehicle_apis.end(); ++it) {
        std::string message;
        if (!it->second->isReady(message)) {
            UAirBlueprintLib::LogMessage(common_utils::Utils::stringf("Vehicle %s was not initialized: ", 
                it->first.c_str()).c_str(), message.c_str(), LogDebugLevel::Failure);
            UAirBlueprintLib::LogMessage("Tip: check connection info in settings.json", "", LogDebugLevel::Informational);
        }
    }
}

void ASimModeBase::setStencilIDs()
{
    UAirBlueprintLib::SetMeshNamingMethod(getSettings().segmentation_setting.mesh_naming_method);

    if (getSettings().segmentation_setting.init_method ==
        AirSimSettings::SegmentationSetting::InitMethodType::CommonObjectsRandomIDs) {
     
        UAirBlueprintLib::InitializeMeshStencilIDs(!getSettings().segmentation_setting.override_existing);
    }
    //else don't init
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FRecordingThread::stopRecording();
    world_sim_api_.reset(nullptr);
    CameraDirector = nullptr;

    Super::EndPlay(EndPlayReason);
}

void ASimModeBase::setupTimeOfDay()
{
    sky_sphere_ = nullptr;

    const auto& tod_setting = getSettings().tod_setting;

    if (tod_setting.enabled) {
        TArray<AActor*> sky_spheres;
        UGameplayStatics::GetAllActorsOfClass(this->GetWorld(), sky_sphere_class_, sky_spheres);
        if (sky_spheres.Num() == 0)
            UAirBlueprintLib::LogMessage(TEXT("BP_Sky_Sphere was not found. "), 
                TEXT("TimeOfDay settings would be ignored."), LogDebugLevel::Failure);
        else if (sky_spheres.Num() > 1)
            UAirBlueprintLib::LogMessage(TEXT("More than BP_Sky_Sphere were found. "), 
                TEXT("TimeOfDay settings would be applied to first one."), LogDebugLevel::Failure);

        if (sky_spheres.Num() >= 1) {
            sky_sphere_ = sky_spheres[0];
            static const FName sun_prop_name(TEXT("Directional light actor"));
            auto* p = sky_sphere_class_->FindPropertyByName(sun_prop_name);
            UObjectProperty* sun_prop = Cast<UObjectProperty>( p);
            UObject* sun_obj = sun_prop->GetObjectPropertyValue_InContainer(sky_sphere_);
            sun_ = Cast<ADirectionalLight>(sun_obj);
            if (sun_) {
                sun_->GetRootComponent()->Mobility = EComponentMobility::Movable;
            }

            tod_sim_clock_start_ = ClockFactory::get()->nowNanos();
            tod_last_update_ = 0;
            if (tod_setting.start_datetime != "")
                tod_start_time_ = Utils::to_time_t(tod_setting.start_datetime, tod_setting.is_start_datetime_dst);
            else
                tod_start_time_ = std::time(nullptr);
        }
    }
    //else ignore
}

bool ASimModeBase::isPaused() const
{
    return false;
}

void ASimModeBase::pause(bool is_paused)
{
    //should be overridden by derived class
    unused(is_paused);
    throw std::domain_error("Pause is not implemented by SimMode");
}

void ASimModeBase::continueForTime(double seconds)
{
    //should be overridden by derived class
    unused(seconds);
    throw std::domain_error("continueForTime is not implemented by SimMode");
}

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeBase::createApiServer() const
{
    //this will be the case when compilation with RPCLIB is disabled or simmode doesn't support APIs
    return nullptr;
}

void ASimModeBase::setupClockSpeed()
{
    //default setup - this should be overridden in derived modes as needed

    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock")
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    else if (clock_type == "SteppableClock")
        ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
            static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}

void ASimModeBase::setupPhysicsLoopPeriod()
{
    /*
    300Hz seems to be minimum for non-aggressive flights
    400Hz is needed for moderately aggressive flights (such as
    high yaw rate with simultaneous back move)
    500Hz is recommended for more aggressive flights
    Lenovo P50 high-end config laptop seems to be topping out at 400Hz.
    HP Z840 desktop high-end config seems to be able to go up to 500Hz.
    To increase freq with limited CPU power, switch Barometer to constant ref mode.
    */

    if (getSettings().usage_scenario == kUsageScenarioComputerVision)
        physics_loop_period_ = 30000000LL; //30ms
    else
        physics_loop_period_ = 3000000LL; //3ms
}

long long ASimModeBase::getPhysicsLoopPeriod() const //nanoseconds
{
    return physics_loop_period_;
}

void ASimModeBase::setPhysicsLoopPeriod(long long  period)
{
    physics_loop_period_ = period;
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (isRecording())
        ++record_tick_count;

    advanceTimeOfDay();

    showClockStats();
    
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::showClockStats()
{
    float clock_speed = getSettings().clock_speed;
    if (clock_speed != 1) {
        UAirBlueprintLib::LogMessageString("ClockSpeed config, actual: ", 
            Utils::stringf("%f, %f", clock_speed, ClockFactory::get()->getTrueScaleWrtWallClock()), 
            LogDebugLevel::Informational);
    }
}

void ASimModeBase::advanceTimeOfDay()
{
    const auto& settings = getSettings();

    if (settings.tod_setting.enabled && sky_sphere_ && sun_) {
        auto secs = ClockFactory::get()->elapsedSince(tod_last_update_);
        if (secs > settings.tod_setting.update_interval_secs) {
            tod_last_update_ = ClockFactory::get()->nowNanos();

            auto interval = ClockFactory::get()->elapsedSince(tod_sim_clock_start_) * settings.tod_setting.celestial_clock_speed;
            uint64_t cur_time = ClockFactory::get()->addTo(tod_sim_clock_start_, interval)  / 1E9;

            UAirBlueprintLib::LogMessageString("DateTime: ", Utils::to_string(cur_time), LogDebugLevel::Informational);

            auto coord = msr::airlib::EarthCelestial::getSunCoordinates(cur_time, settings.origin_geopoint.home_geo_point.latitude,
                settings.origin_geopoint.home_geo_point.longitude);

            auto rot = FRotator(-coord.altitude, coord.azimuth, 0);
            sun_->SetActorRotation(rot);

            FOutputDeviceNull ar;
            sky_sphere_->CallFunctionByNameWithArguments(TEXT("UpdateSunDirection"), ar, NULL, true);
        }
         
    }
}

void ASimModeBase::reset()
{
    //Should be overridden by derived classes
}

std::string ASimModeBase::getDebugReport()
{
    static const std::string empty_string = std::string();
    //Should be overridden by derived classes
    return empty_string;
}

void ASimModeBase::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("InputEventResetAll", EKeys::BackSpace, this, &ASimModeBase::reset);
}

ECameraDirectorMode ASimModeBase::getInitialViewMode() const
{
    return Utils::toEnum<ECameraDirectorMode>(getSettings().initial_view_mode);
}

const msr::airlib::AirSimSettings& ASimModeBase::getSettings() const
{
    return AirSimSettings::singleton();
}

bool ASimModeBase::toggleRecording()
{
    if (isRecording())
        stopRecording();
    else
        startRecording();

    return isRecording();
}

void ASimModeBase::stopRecording()
{
    FRecordingThread::stopRecording();
}

void ASimModeBase::startRecording()
{
    FRecordingThread::startRecording(getVehicleSimApi()->getImageCapture(),
        getVehicleSimApi()->getGroundTruthKinematics(), getSettings().recording_setting ,
        getVehicleSimApi());
}

bool ASimModeBase::isRecording() const
{
    return FRecordingThread::isRecording();
}

//API server start/stop
void ASimModeBase::startApiServer()
{
    if (getSettings().enable_rpc) {

#ifdef AIRLIB_NO_RPC
        api_server_.reset();
#else
        api_provider_.reset(new msr::airlib::ApiProvider<VehicleSimApi>(world_sim_api_.get()));
        api_server_ = createApiServer();
#endif

        try {
            api_server_->start();
        }
        catch (std::exception& ex) {
            UAirBlueprintLib::LogMessageString("Cannot start RpcLib Server", ex.what(), LogDebugLevel::Failure);
        }
    }
    else
        UAirBlueprintLib::LogMessageString("API server is disabled in settings", "", LogDebugLevel::Informational);

}
void ASimModeBase::stopApiServer()
{
    if (api_server_ != nullptr) {
        api_server_->stop();
        api_server_.reset(nullptr);
        api_provider_.reset(nullptr);
    }
}
bool ASimModeBase::isApiServerStarted()
{
    return api_server_ != nullptr;
}
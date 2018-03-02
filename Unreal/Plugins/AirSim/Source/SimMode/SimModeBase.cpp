#include "SimModeBase.h"
#include <memory>
#include "Misc/MessageDialog.h"
#include "Misc/EngineVersion.h"
#include "AirBlueprintLib.h"
#include "Runtime/Launch/Resources/Version.h"
#include "common/AirSimSettings.hpp"
#include "Recording/RecordingThread.h"
#include "common/ScalableClock.hpp"
#include "common/SteppableClock.hpp"
#include "ConstructorHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "SimJoyStick/SimJoyStick.h"
#include "Misc/OutputDeviceNull.h"
#include "common/EarthCelestial.hpp"


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

    setupClock();

    setStencilIDs();
    
    record_tick_count = 0;
    setupInputBindings();

    setupTimeOfDay();

    UAirBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""), LogDebugLevel::Informational);
}

void ASimModeBase::setStencilIDs()
{
    UAirBlueprintLib::SetMeshNamingMethod(getSettings().segmentation_settings.mesh_naming_method);

    if (getSettings().segmentation_settings.init_method ==
        AirSimSettings::SegmentationSettings::InitMethodType::CommonObjectsRandomIDs) {
     
        UAirBlueprintLib::InitializeMeshStencilIDs(!getSettings().segmentation_settings.override_existing);
    }
    //else don't init
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    FRecordingThread::stopRecording();
    Super::EndPlay(EndPlayReason);
}

void ASimModeBase::setupTimeOfDay()
{
    sky_sphere_ = nullptr;

    const auto& tod_settings = getSettings().tod_settings;

    if (tod_settings.enabled) {
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
            if (tod_settings.start_datetime != "")
                tod_start_time_ = Utils::to_time_t(tod_settings.start_datetime, tod_settings.is_start_datetime_dst);
            else
                tod_start_time_ = std::time(nullptr);
        }
    }
    //else ignore
}


void ASimModeBase::setupClock()
{
    float clock_speed = getSettings().clock_speed;

    //setup clock in PhysX
    if (clock_speed != 1.0f) {
        this->GetWorldSettings()->SetTimeDilation(clock_speed);
        UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(clock_speed), LogDebugLevel::Informational);
    }

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

long long ASimModeBase::getPhysicsLoopPeriod() //nanoseconds
{
    /*
    300Hz seems to be minimum for non-aggresive flights
    400Hz is needed for moderately aggressive flights (such as
    high yaw rate with simultaneous back move)
    500Hz is recommanded for more aggressive flights
    Lenovo P50 high-end config laptop seems to be topping out at 400Hz.
    HP Z840 desktop high-end config seems to be able to go up to 500Hz.
    To increase freq with limited CPU power, switch Barometer to constant ref mode.
    */

    if (getSettings().usage_scenario == kUsageScenarioComputerVision)
        return 30000000LL; //30ms
    else
        return 3000000LL; //3ms
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    if (isRecording())
        ++record_tick_count;

    advanceTimeOfDay();

    Super::Tick(DeltaSeconds);
}

void ASimModeBase::advanceTimeOfDay()
{
    const auto& settings = getSettings();

    if (settings.tod_settings.enabled && sky_sphere_ && sun_) {
        auto secs = ClockFactory::get()->elapsedSince(tod_last_update_);
        if (secs > settings.tod_settings.update_interval_secs) {
            tod_last_update_ = ClockFactory::get()->nowNanos();

            auto interval = ClockFactory::get()->elapsedSince(tod_sim_clock_start_) * settings.tod_settings.celestial_clock_speed;
            uint64_t cur_time = ClockFactory::get()->addTo(tod_sim_clock_start_, interval)  / 1E9;

            UAirBlueprintLib::LogMessageString("DateTime: ", Utils::to_string(cur_time), LogDebugLevel::Informational);

            auto coord = msr::airlib::EarthCelestial::getSunCoordinates(cur_time, settings.origin_geopoint.home_point.latitude,
                settings.origin_geopoint.home_point.longitude);

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

VehiclePawnWrapper* ASimModeBase::getFpvVehiclePawnWrapper()
{
    //Should be overridden by derived classes
    return nullptr;
}


std::string ASimModeBase::getReport()
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

bool ASimModeBase::isRecording()
{
    return FRecordingThread::isRecording();
}

bool ASimModeBase::isRecordUIVisible()
{
    return getSettings().is_record_ui_visible;
}

ECameraDirectorMode ASimModeBase::getInitialViewMode()
{
    return Utils::toEnum<ECameraDirectorMode>(getSettings().initial_view_mode);
}

void ASimModeBase::startRecording()
{
    FRecordingThread::startRecording(getFpvVehiclePawnWrapper()->getImageCapture(),
        getFpvVehiclePawnWrapper()->getTrueKinematics(), getSettings().recording_settings, getFpvVehiclePawnWrapper());
}

const AirSimSettings& ASimModeBase::getSettings() const
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


#include "SimModeWorldBase.h"
#include <exception>
#include "AirBlueprintLib.h"


void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();
}

void ASimModeWorldBase::initializeForPlay()
{
    std::vector<msr::airlib::UpdatableObject*> vehicles;
    for (auto& api : getApiProvider()->getVehicleSimApis())
        vehicles.push_back(api);
    //TODO: directly accept getVehicleSimApis() using generic container

    std::unique_ptr<PhysicsEngineBase> physics_engine = createPhysicsEngine();
    physics_engine_ = physics_engine.get();
    physics_world_.reset(new msr::airlib::PhysicsWorld(std::move(physics_engine),
        vehicles, getPhysicsLoopPeriod()));
}

void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //remove everything that we created in BeginPlay
    physics_world_.reset();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldBase::startAsyncUpdator()
{
    physics_world_->startAsyncUpdator();
}
void ASimModeWorldBase::stopAsyncUpdator()
{
    physics_world_->stopAsyncUpdator();
}

long long ASimModeWorldBase::getPhysicsLoopPeriod() const //nanoseconds
{
    return physics_loop_period_;
}
void ASimModeWorldBase::setPhysicsLoopPeriod(long long  period)
{
    physics_loop_period_ = period;
}

std::unique_ptr<ASimModeWorldBase::PhysicsEngineBase> ASimModeWorldBase::createPhysicsEngine()
{
    std::unique_ptr<PhysicsEngineBase> physics_engine;
    std::string physics_engine_name = getSettings().physics_engine_name;
    if (physics_engine_name == "")
        physics_engine.reset(); //no physics engine
    else if (physics_engine_name == "FastPhysicsEngine") {
        msr::airlib::Settings fast_phys_settings;
        if (msr::airlib::Settings::singleton().getChild("FastPhysicsEngine", fast_phys_settings)) {
            physics_engine.reset(new msr::airlib::FastPhysicsEngine(fast_phys_settings.getBool("EnableGroundLock", true)));
        }
        else {
            physics_engine.reset(new msr::airlib::FastPhysicsEngine());
        }
    }
    else {
        physics_engine.reset();
        UAirBlueprintLib::LogMessageString("Unrecognized physics engine name: ",  physics_engine_name, LogDebugLevel::Failure);
    }

    return physics_engine;
}

bool ASimModeWorldBase::isPaused() const
{
    return physics_world_->isPaused();
}

void ASimModeWorldBase::pause(bool is_paused)
{
    physics_world_->pause(is_paused);
}

void ASimModeWorldBase::continueForTime(double seconds)
{
    physics_world_->continueForTime(seconds);

}

void ASimModeWorldBase::updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter)
{
    unused(debug_reporter);
    //we use custom debug reporting for this class
}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
    { //keep this lock as short as possible
        physics_world_->lock();

        physics_world_->enableStateReport(EnableReport);
        physics_world_->updateStateReport();

        for (auto& api : getApiProvider()->getVehicleSimApis())
            api->updateRenderedState(DeltaSeconds);

        physics_world_->unlock();
    }

    //perform any expensive rendering update outside of lock region
    for (auto& api : getApiProvider()->getVehicleSimApis())
        api->updateRendering(DeltaSeconds);

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldBase::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        physics_world_->reset();
    }, true);
    
    //no need to call base reset because of our custom implementation
}

std::string ASimModeWorldBase::getDebugReport()
{
	// Most of the report comes from AirLib, but this one bit, the main camera's LLA, comes from Unreal.  That's why it's prepended here.
	// Used to show current LLA - base it on the origin
	NedTransform ned_transform_(FTransform::Identity, UAirBlueprintLib::GetWorldToMetersScale(this));

	// Get main camera position
	FTransform main_camera_transform = GetWorld()->GetFirstPlayerController()->GetViewTarget()->GetActorTransform();
	FVector cameraLocation = main_camera_transform.GetLocation();

	// Transform to LLA and log
	const auto& settings = AirSimSettings::singleton();
	msr::airlib::Vector3r ned = ned_transform_.toGlobalNed(cameraLocation);
	msr::airlib::GeoPoint lla = msr::airlib::EarthUtils::nedToGeodetic(ned, settings.origin_geopoint.home_geo_point);

	std::string prefix = "Main camera LLA: ";
	prefix.append(lla.to_string()).append("\n");
	
	return prefix + physics_world_->getDebugReport();
}

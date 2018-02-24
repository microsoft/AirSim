#include "SimModeWorldBase.h"
#include <exception>


void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();

    manual_pose_controller = NewObject<UManualPoseController>();
    setupInputBindings();

    //call virtual method in derived class
    createVehicles(vehicles_);

    physics_world_.reset(new msr::airlib::PhysicsWorld(
        createPhysicsEngine(), toUpdatableObjects(vehicles_), 
        getPhysicsLoopPeriod()));

    if (getSettings().usage_scenario == kUsageScenarioComputerVision) {
        if (getSettings().default_vehicle_config != "SimpleFlight")
            UAirBlueprintLib::LogMessageString("settings.json is not using simple_flight in ComputerVision mode."
                "This can lead to unpredictable behaviour!",  
                "", LogDebugLevel::Failure);


        manual_pose_controller->initializeForPlay();
        manual_pose_controller->setActor(getFpvVehiclePawnWrapper()->getPawn());
    }
}


void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //remove everything that we created in BeginPlay
    physics_world_.reset();
    physics_engine_.reset();
    vehicles_.clear();
    manual_pose_controller = nullptr;

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


std::vector<ASimModeWorldBase::UpdatableObject*> ASimModeWorldBase::toUpdatableObjects(
    const std::vector<ASimModeWorldBase::VehiclePtr>& vehicles)
{
    std::vector<UpdatableObject*> bodies;
    for (const VehiclePtr& body : vehicles)
        bodies.push_back(body.get());

    return bodies;
}


ASimModeWorldBase::PhysicsEngineBase* ASimModeWorldBase::createPhysicsEngine()
{
    std::string physics_engine_name = getSettings().physics_engine_name;
    if (physics_engine_name == "" || getSettings().usage_scenario == kUsageScenarioComputerVision)
        physics_engine_.reset(); //no physics engine
    else if (physics_engine_name == "FastPhysicsEngine") {
        msr::airlib::Settings fast_phys_settings;
        if (msr::airlib::Settings::singleton().getChild("FastPhysicsEngine", fast_phys_settings)) {
            physics_engine_.reset(
                new msr::airlib::FastPhysicsEngine(fast_phys_settings.getBool("EnableGroundLock", true))
            );
        }
        else {
            physics_engine_.reset(
                new msr::airlib::FastPhysicsEngine()
            );
        }
    }
    else {
        physics_engine_.reset();
        UAirBlueprintLib::LogMessageString("Unrecognized physics engine name: ",  physics_engine_name, LogDebugLevel::Failure);
    }

    return physics_engine_.get();
}

size_t ASimModeWorldBase::getVehicleCount() const
{
    return vehicles_.size();
}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
    { //keep this lock as short as possible
        physics_world_->lock();

        physics_world_->enableStateReport(EnableReport);
        physics_world_->updateStateReport();

        for (auto& vehicle : vehicles_)
            vehicle->updateRenderedState(DeltaSeconds);

        physics_world_->unlock();
    }

    //perfom any expensive rendering update outside of lock region
    for (auto& vehicle : vehicles_)
        vehicle->updateRendering(DeltaSeconds);

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldBase::reset()
{
    physics_world_->reset();
    
    Super::reset();
}

std::string ASimModeWorldBase::getReport()
{
    return physics_world_->getReport();
}

void ASimModeWorldBase::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    //should be overridden by derived class
    //Unreal doesn't allow pure abstract methods in actors
}


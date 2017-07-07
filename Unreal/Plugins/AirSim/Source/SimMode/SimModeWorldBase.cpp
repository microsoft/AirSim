#include "AirSim.h"
#include "SimModeWorldBase.h"


void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();

    manual_pose_controller = NewObject<UManualPoseController>();
    setupInputBindings();

    //call virtual method in derived class
    createVehicles(vehicles_);

    createWorld();

    /*
    300Hz seems to be minimum for non-aggresive flights
    400Hz is needed for moderately aggressive flights (such as
    high yaw rate with simultaneous back move)
    500Hz is recommanded for more aggressive flights
    Lenovo P50 high-end config laptop seems to be topping out at 400Hz.
    HP Z840 desktop high-end config seems to be able to go up to 500Hz.
    To increase freq with limited CPU power, switch Barometer to constant ref mode.
    */
   
    if (usage_scenario == kUsageScenarioComputerVision) {
        world_.startAsyncUpdator(30000000LL);

        manual_pose_controller->initializeForPlay();
        manual_pose_controller->setActor(getFpvVehiclePawn());
    }
    else
        world_.startAsyncUpdator(3000000LL);
}

void ASimModeWorldBase::createWorld()
{
    if (physics_engine_name == "" || usage_scenario == kUsageScenarioComputerVision)
        physics_engine_.release(); //no physics engine
    else if (physics_engine_name == "FastPhysicsEngine")
        physics_engine_.reset(new msr::airlib::FastPhysicsEngine());
    else {
        physics_engine_.release();
        UAirBlueprintLib::LogMessageString("Unrecognized physics engine name: ",  physics_engine_name, LogDebugLevel::Failure);
    }

    world_.initialize(physics_engine_.get());
    reporter_.initialize(false);

    //add default objects to world
    world_.insert(&reporter_);
    for(size_t vi = 0; vi < vehicles_.size(); vi++)
        world_.insert(vehicles_.at(vi).get());

    for (auto& vehicle : vehicles_)
        vehicle->beginPlay();
}

size_t ASimModeWorldBase::getVehicleCount() const
{
    return vehicles_.size();
}


void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    world_.stopAsyncUpdator();

    for (auto& vehicle : vehicles_)
        vehicle->endPlay();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
    world_.lock();

    for (auto& vehicle : vehicles_)
        vehicle->updateRenderedState();

    reporter_.setEnable(EnableReport);
    if (reporter_.canReport()) {
        reporter_.clearReport();
        world_.reportState(*reporter_.getReporter());
    }

    world_.unlock();

    //perfom any expensive rendering update outside of lock region
    for (auto& vehicle : vehicles_)
        vehicle->updateRendering(DeltaSeconds);

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldBase::reset()
{
    world_.lock();
    world_.reset();
    world_.unlock();

    Super::reset();
}

std::string ASimModeWorldBase::getReport()
{
    return reporter_.getOutput();
}

void ASimModeWorldBase::createVehicles(std::vector<VehiclePtr>& vehicles)
{
    //should be overridden by derived class
    //Unreal doesn't allow pure abstract methods in actors
}

void ASimModeWorldBase::setupInputBindings()
{
    Super::setupInputBindings();

    UAirBlueprintLib::BindActionToKey("InputEventResetAll", EKeys::BackSpace, this, &ASimModeWorldBase::reset);
}
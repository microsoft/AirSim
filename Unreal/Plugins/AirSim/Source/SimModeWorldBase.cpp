#include "AirSim.h"
#include "SimModeWorldBase.h"


void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();

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
    world_.startAsyncUpdator(1/500.0f);
}

void ASimModeWorldBase::createWorld()
{
    world_.initialize(&physics_engine_);
    reporter_.initialize(false);

    //add default objects to world
    world_.insert(&reporter_);
    for(size_t vi = 0; vi < vehicles_.size(); vi++)
        world_.insert(vehicles_.at(vi).get());

    world_.setCurrentMembersAsInitial();

    for (auto& vehicle : vehicles_)
        vehicle->beginPlay();
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
        vehicle->updateRendering();

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

    UAirBlueprintLib::BindActionTokey("InputEventResetAll", EKeys::BackSpace, this, &ASimModeWorldBase::reset);
}
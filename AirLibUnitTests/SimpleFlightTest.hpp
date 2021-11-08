
#ifndef msr_AirLibUnitTests_SimpleFlightTest_hpp
#define msr_AirLibUnitTests_SimpleFlightTest_hpp

#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "TestBase.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/SteppableClock.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"

#include "settings_json_parser.hpp"

#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightQuadXParams.hpp"

namespace msr
{
namespace airlib
{

    class SimpleFlightTest : public TestBase
    {
    public:
        virtual void run() override
        {
            auto clock = std::make_shared<SteppableClock>(3E-3f);
            ClockFactory::get(clock);

            SensorFactory sensor_factory;
            // added by Suman, from https://github.com/microsoft/AirSim/pull/2558/commits/9c4e59d1a2b371ebc60cdc18f93b06cbe3e9d305
            SettingsLoader settings_loader;

            std::unique_ptr<MultiRotorParams> params = MultiRotorParamsFactory::createConfig(
                AirSimSettings::singleton().getVehicleSetting("SimpleFlight"),
                std::make_shared<SensorFactory>());
            auto api = params->createMultirotorApi();

            std::unique_ptr<msr::airlib::Kinematics> kinematics;
            std::unique_ptr<msr::airlib::Environment> environment;
            Kinematics::State initial_kinematic_state = Kinematics::State::zero();
            
            initial_kinematic_state.pose = Pose();
            kinematics.reset(new Kinematics(initial_kinematic_state));

            Environment::State initial_environment;
            initial_environment.position = initial_kinematic_state.pose.position;
            initial_environment.geo_point = GeoPoint();
            environment.reset(new Environment(initial_environment));

            MultiRotorPhysicsBody vehicle(params.get(), api.get(), kinematics.get(), environment.get());

            std::vector<UpdatableObject*> vehicles = { &vehicle };
            std::unique_ptr<PhysicsEngineBase> physics_engine(new FastPhysicsEngine());
            PhysicsWorld physics_world(std::move(physics_engine), vehicles, static_cast<uint64_t>(clock->getStepSize() * 1E9));
            // world.startAsyncUpdator(); called in the physics_world constructor

            // added by Suman, to fix calling update before reset https://github.com/microsoft/AirSim/issues/2773#issuecomment-703888477
            // TODO not sure if it should be here? see wrt to PawnSimApi
            api->setSimulatedGroundTruth(&kinematics->getState(), environment.get());
            api->reset();
            kinematics->reset();

            StateReporter reporter;
            kinematics->reportState(reporter); // this writes the kinematics in reporter
            std::cout << reporter.getOutput() << std::endl;


            testAssert(api != nullptr, "api was null");
            std::string message;
            testAssert(api->isReady(message), message);

            clock->sleep_for(0.04f);

            Utils::getSetMinLogLevel(true, 100);

            api->enableApiControl(true);
            api->armDisarm(true);
            api->takeoff(10);

            // Vector3r pos = api->getMultirotorState().getPosition();
            // std::cout << pos << std::endl;

            kinematics->reportState(reporter); // this writes the kinematics in reporter
            std::cout << reporter.getOutput() << std::endl;

            // TODO print some values OR log

            clock->sleep_for(2.0f);

            Utils::getSetMinLogLevel(true);

            //api->moveToPosition(-5, -5, -5, 5, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            // pos = api->getMultirotorState().getPosition();
            // std::cout << pos << std::endl;

            clock->sleep_for(2.0f);

            while (true) {
                clock->sleep_for(0.1f);
                api->getStatusMessages(messages_);
                for (const auto& status_message : messages_) {
                    std::cout << status_message << std::endl;
                }
                messages_.clear();
            }
        }

    private:
        std::vector<std::string> messages_;
    };
}
}
#endif
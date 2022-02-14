
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
    void checkStatusMsg(MultirotorApiBase* api, std::ofstream* myfile)
    {

        std::vector<std::string> messages_;
        api->getStatusMessages(messages_);
        for (const auto& status_message : messages_) {
            *myfile << status_message << std::endl;
            // std::cout << status_message << std::endl;
        }
    }

    class SimpleFlightTest : public TestBase
    {
    public:
        virtual void run() override
        {
            std::cout << std::endl;
            auto clock = std::make_shared<SteppableClock>(3E-3f);
            // auto clock = std::make_shared<SteppableClock>(1E-1f);
            ClockFactory::get(clock);

            SensorFactory sensor_factory;

            // added from https://github.com/microsoft/AirSim/pull/2558/commits/9c4e59d1a2b371ebc60cdc18f93b06cbe3e9d305
            // loads settings from settings.json or Default setting
            SettingsLoader settings_loader;

            std::unique_ptr<MultiRotorParams> params = MultiRotorParamsFactory::createConfig(
                AirSimSettings::singleton().getVehicleSetting("SimpleFlight"),
                std::make_shared<SensorFactory>());
            auto api = params->createMultirotorApi();

            // create and initialize kinematics and environment
            std::unique_ptr<msr::airlib::Kinematics> kinematics;
            std::unique_ptr<msr::airlib::Environment> environment;
            Kinematics::State initial_kinematic_state = Kinematics::State::zero();
            
            initial_kinematic_state.pose = Pose();
            // states_(6) = 0.9961946f; // q0
            // states_(7) = 0.0f; // q1
            // states_(8) = 0.08715574f; // q2

            // initial_kinematic_state.pose.orientation.w() = 0.9961946f;
            // initial_kinematic_state.pose.orientation.x() = 0.0f;
            // initial_kinematic_state.pose.orientation.y() = 0.08715574f;
            // initial_kinematic_state.pose.orientation.z() = 0.0f;

            // initial_kinematic_state.pose.position.z() = -10.0f;
            kinematics.reset(new Kinematics(initial_kinematic_state));

            Environment::State initial_environment;
            initial_environment.position = initial_kinematic_state.pose.position;
            initial_environment.geo_point = GeoPoint();
            initial_environment.geo_point.altitude = 0.0f; // do not set it equal to kinematics z, this value goes into home geo point and acts as the ref for kinematics z
            environment.reset(new Environment(initial_environment));

            // crete and initialize body and physics world
            MultiRotorPhysicsBody vehicle(params.get(), api.get(), kinematics.get(), environment.get()); 

            std::vector<UpdatableObject*> vehicles = { &vehicle };
            std::unique_ptr<PhysicsEngineBase> physics_engine(new FastPhysicsEngine());
            PhysicsWorld physics_world(std::move(physics_engine), vehicles);//, static_cast<uint64_t>(clock->getStepSize() * 1E9));
            // world.startAsyncUpdator(); called in the physics_world constructor

            // added by Suman, to fix calling update before reset https://github.com/microsoft/AirSim/issues/2773#issuecomment-703888477
            // TODO not sure if it should be here? see wrt to PawnSimApi, no side effects so far
            api->setSimulatedGroundTruth(&kinematics.get()->getState(), environment.get());
            api->reset();
            kinematics->reset();

            // set the vehicle as grounded, otherwise can not take off, needs to to be done after physics world construction!
            vehicle.setGrounded(true);

            // read intitial position
            Vector3r pos = api->getMultirotorState().getPosition();
            std::cout << "starting position: " << pos << std::endl;

            // test api if not null
            testAssert(api != nullptr, "api was null");
            std::string message;
            testAssert(api->isReady(message), message);


            Utils::getSetMinLogLevel(true, 100);

            std::ostringstream ss;

            std::ofstream myfile;
            myfile.open("log.txt");
            /*myfile << ">> Physics update frequency: 333.33 Hz.\n";
            myfile << ">> Barometer and magnetometer update frequency: 50 Hz.\n";
            myfile << ">> GPS update frequency: 50 Hz with startup delay.\n\n";*/
            myfile << ">> timestamp (ms) \t GroundTruth altitude \t Estimated postiion (x,y,z) \n\n";
            
            // enable api control
            api->enableApiControl(true);
            //checkStatusMsg(api.get(), &myfile);

            // arm
            api->armDisarm(true);
            //checkStatusMsg(api.get(), &myfile);

            clock->sleep_for(30.0f);

            // take off
            api->takeoff(50);
            pos = api->getMultirotorState().getPosition();
            std::cout << "took-off position: " << pos << std::endl;
            //checkStatusMsg(api.get(), &myfile);

            // clock->sleep_for(60.0f);
            // api->resetEkf();

            // api->commandAngleRatesZ(0.0f, 0.0f, 0.1f, 0.0f);
            // clock->sleep_for(18.0f);
            // api->moveByAngleRatesZ(0.0f, 0.0f, 9.0f*M_PI/180, -1.53509f, 10.0f);
            // clock->sleep_for(18.0f);
            // api->moveByAngleRatesZ(0.0f, 4.0f*M_PI/180, 0.0f, -1.53509f, 10.0f);
            // clock->sleep_for(60.0f);
            // api->moveByAngleRatesZ(0.0f, 4.0f*M_PI/180, 0.0f, -1.53509f, 10.0f);
            // fly towards a waypoint
            // api->moveToPosition(-1, 0, -1.53509, 0.1, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            // pos = api->getMultirotorState().getPosition();
            // std::cout << "waypoint position: " << pos << std::endl;
            // api->moveToPosition(0, -1, -1.53509, 0.1, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            // pos = api->getMultirotorState().getPosition();
            //checkStatusMsg(api.get(), &myfile);

            clock->sleep_for(10.0f);
            api->moveByAngleRatesZ(0.0f,  0.0f, 4.0f*M_PI/180, -1.53509f, 4.0f);
            clock->sleep_for(10.0f);
            // // // fly towards a waypoint
            api->moveToPosition(10, 0, -2, 0.5, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            clock->sleep_for(10.0f);
            api->moveToPosition(10, 10, -2, 0.5, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            clock->sleep_for(10.0f);
            api->moveToPosition(10, 10, -20, 0.5, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            pos = api->getMultirotorState().getPosition();
            std::cout << "waypoint position: " << pos << std::endl;
            // // api->commandVelocity(10.0f, 0.0f, 0.0f, YawMode(true, 0));
            // // pos = api->getMultirotorState().getPosition();
            // // //std::cout << "waypoint position: " << pos << std::endl;
            // // //checkStatusMsg(api.get(), &myfile);

            // clock->sleep_for(60.0f);
            // api->moveToPosition(-100, 100, -50, 15, 1E3, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);
            // pos = api->getMultirotorState().getPosition();
            // std::cout << "waypoint position: " << pos << std::endl;
            clock->sleep_for(20.0f);

            // // land
            // //api->land(10);
            // pos = api->getMultirotorState().getPosition();
            // std::cout << "final    position: " << pos << std::endl;
            checkStatusMsg(api.get(), &myfile);

            // TODO print some values OR log

            // // report states
            // std::cout << std::endl;
            // StateReporter reporter;
            // kinematics->reportState(reporter); // this writes the kinematics in reporter
            // std::cout << reporter.getOutput() << std::endl;
            
            myfile.close();

            /*while (true) {
            }*/

        }

    private:
        //std::vector<std::string> messages_;
    };
}
}
#endif
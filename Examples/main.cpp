#include "StandAloneSensors.hpp"
#include "StandAlonePhysics.hpp"
#include "StereoImageGenerator.hpp"
#include "GaussianMarkovTest.hpp"
#include "DepthNav/DepthNavCost.hpp"
#include "DepthNav/DepthNavOptAStar.hpp"
#include <iostream>
#include <string>

int runStandAloneSensors(int argc, const char *argv[])
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <out_file_name> <period_ms> <total_duration_sec>" << std::endl;
        return 1;
    }

    float period = 30E-3f;
    if (argc >= 3)
        period = std::stof(argv[2]) * 1E-3f;

    float total_duration = 3600;
    if (argc >= 4)
        total_duration = std::stof(argv[3]);

    std::cout << "Period is " << period << "sec" << std::endl;
    std::cout << "Total duration is " << total_duration << "sec" << std::endl;


    using namespace msr::airlib;

    //60 acres park:
    //GeoPoint testLocation(47.7037051477, -122.1415384809, 9.93f); 

    //marymoore park
    //GeoPoint testLocation(47.662804385, -122.1167039875, 9.93f);

    GeoPoint testLocation(47.7631699747, -122.0685655406, 9.93f); // woodinville
    float yawOffset = 0;// static_cast<float>(91.27622  * M_PI / 180.0); // I was aligned with the road...

    std::ofstream out_file(argv[1]);
    StandALoneSensors::generateImuStaticData(out_file, period, total_duration);
    StandALoneSensors::generateBarometerStaticData(out_file, period, total_duration, testLocation);
    StandALoneSensors::generateBarometerDynamicData(out_file, period, total_duration, testLocation);
    StandALoneSensors::generateMagnetometer2D(out_file, period, total_duration, testLocation, yawOffset, true);
    StandALoneSensors::generateMagnetometerMap(out_file);

    return 0;
}

int runStandAlonePhysics(int argc, const char *argv[])
{
    using namespace msr::airlib;

    StandAlonePhysics::testCollison();

    return 0;
}

void runSteroImageGenerator(int num_samples, std::string storage_path)
{
    StereoImageGenerator gen(storage_path);
    gen.generate(num_samples);
}

void runSteroImageGenerator(int argc, const char *argv[])
{
    runSteroImageGenerator(argc < 2 ? 50000 : std::stoi(argv[1]), argc < 3 ? 
        common_utils::FileSystem::combine(
            common_utils::FileSystem::getAppDataFolder(), "stereo_gen")
        : std::string(argv[2]));
}

void runGaussianMarkovTest()
{
	using namespace msr::airlib;

	GaussianMarkovTest test;
	test.run();
}

void runDepthNavTest()
{
    Pose startPose = Pose(Vector3r(0, 0, -1), Quaternionr(1, 0, 0, 0)); //start pose
    //Pose goalPose = Pose(Vector3r(50, 20, -1), Quaternionr(1, 0, 0, 0)); //final pose
    Pose goalPose = Pose(Vector3r(50, 105, -1), Quaternionr(1, 0, 0, 0)); //final pose

    RpcLibClientBase client;
    client.confirmConnection();

    client.simSetVehiclePose(startPose, true);
    std::cout << "Press Enter to start" << std::endl; std::cin.get(); //Allow some time to reach startPose

    //DepthNavThreshold depthNav;
    //DepthNavCost depthNav;
    DepthNavOptAStar depthNav;
    depthNav.gotoGoal(goalPose, client);
}

int main(int argc, const char *argv[])
{
    runDepthNavTest();
    
	return 0;
}
#include "StandAloneSensors.hpp"
#include "StandAlonePhysics.hpp"
#include "StereoImageGenerator.hpp"
#include "GaussianMarkovTest.hpp"
#include "DepthNav.hpp"
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

int main(int argc, const char *argv[])
{

	using namespace msr::airlib;

	//GaussianMarkovTest test;
	//test.run();
	DepthNav depthNav;

	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	typedef common_utils::FileSystem FileSystem;

	MultirotorRpcLibClient client;

	Pose startPose = Pose(Vector3r(0, 0, -1), Quaternionr(1, 0, 0, 0)); //start pose
	Pose currentPose;
	Pose goalPose = Pose(Vector3r(50, 20, -1), Quaternionr(1, 0, 0, 0)); //final pose

	try {
		client.confirmConnection();
		client.reset();
		client.simSetVehiclePose(startPose, true);
		currentPose = startPose;

		bool bGoalReached = false;

		while (bGoalReached != true) {

			std::vector<ImageRequest> request = {
				ImageRequest("1", ImageType::DepthPlanner, true),
				ImageRequest("1", ImageType::Scene),
				ImageRequest("1", ImageType::DisparityNormalized, true)
			};

			const std::vector<ImageResponse>& response = client.simGetImages(request);

			if (response.size() == 0) {
				std::cout << "No images recieved!" << std::endl;
				continue;
			}
			else {
				//std::cout << "# of images recieved: " << response.size() << std::endl;
			}

			for (const ImageResponse& image_info : response) {
				if (image_info.image_type == ImageType::DepthPlanner)
				{
					if (image_info.image_data_float.size() > 0)
					{
						std::vector<float> img;

						for (int i = 0; i<image_info.image_data_float.size();i++) { img.push_back(image_info.image_data_float.data()[i]); }

						currentPose = depthNav.getNextPose(img, goalPose.position, currentPose, 0.5f);
						//std::cout << "Position: " << currentPose.position << " Orientation: " << currentPose.orientation << std::endl;

						if (VectorMath::hasNan(currentPose)) {
							std::cout << "I'm stuck." << std::endl; std::cin.get(); return 0; 
						}
						else {
							client.simSetVehiclePose(currentPose, true);
						}

						float dist2goal = depthNav.getDistanceToGoal(currentPose.position, goalPose.position);

						if (dist2goal < 1) {
							std::cout << "Target reached." << std::endl; std::cin.get();
							return 0;
						}
						else {
							std::cout << "Distance to target: " << dist2goal << std::endl;
						}


					}
				}
			}
		}
	}

	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
		//Add some sleep
		std::this_thread::sleep_for(std::chrono::duration<double>(5));
	}

	return 0;
}
#include "StandAloneSensors.hpp"
#include <iostream>
#include <string>

int main(int argc, const char *argv[])
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

    GeoPoint testLocation(47.763160, -122.068534, 120.6f);

    std::ofstream out_file(argv[1]);
    //StandALoneSensors::createStaticData(out_file, period, total_duration, testLocation);
    StandALoneSensors::generateBarometerStaticData(out_file, period, total_duration, testLocation);
    //StandALoneSensors::generateBarometerDynamicData(out_file, period, total_duration, testLocation);
    //StandALoneSensors::generateMagnetometerDataLoc(out_file, period, total_duration, testLocation);
}
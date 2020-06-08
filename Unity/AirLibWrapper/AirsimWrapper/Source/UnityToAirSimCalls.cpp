#include "UnityToAirSimCalls.h"

void StartServerThread(std::string vehicle_name, std::string sim_mode_name, int port_number)
{
	key = new SimHUD(vehicle_name, sim_mode_name, port_number);
	key->BeginPlay();
}
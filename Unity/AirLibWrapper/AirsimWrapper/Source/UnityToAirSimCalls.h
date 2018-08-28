//#pragma once

#include <thread>
#include "UnityUtilities.hpp"
#include "SimHUD/SimHUD.h"

static SimHUD* key = nullptr;

void StartServerThread(std::string vehicle_name, std::string sim_mode_name, int port_number)
{
	key = new SimHUD(vehicle_name, sim_mode_name, port_number);
	key->BeginPlay();
}

extern "C" __declspec(dllexport) bool StartServer(char* vehicle_name, char* sim_mode_name, int port_number)
{
	std::thread server_thread(StartServerThread, vehicle_name, sim_mode_name, port_number);
	server_thread.detach();
	int waitCounter = 25; // waiting for maximum 5 seconds to start a server.
	while ((key == nullptr || !key->server_started_Successfully_) && waitCounter > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		waitCounter--;
	}
	return key->server_started_Successfully_;
}

extern "C" __declspec(dllexport) void StopServer(char* vehicle_name)
{
	key->EndPlay();
	if (key != nullptr)
	{
		delete key;
		key = nullptr;
	}
}

extern "C" __declspec(dllexport) void CallTick(float deltaSeconds)
{
	key->Tick(deltaSeconds);
}

extern "C" __declspec(dllexport) void InvokeCollisionDetection(AirSimUnity::AirSimCollisionInfo collision_info)
{
	auto simMode = key->GetSimMode();
	if (simMode)
	{
		auto vehicleApi = simMode->getVehicleSimApi(simMode->vehicle_name_);
		if (vehicleApi)
		{
			msr::airlib::CollisionInfo collisionInfo = UnityUtilities::Convert_to_AirSimCollisioinInfo(collision_info);
			vehicleApi->OnCollision(collisionInfo);
		}
	}
}
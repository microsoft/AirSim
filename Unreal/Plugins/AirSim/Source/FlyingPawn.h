#pragma once

#include <memory>
#include "controllers/MavLinkDroneController.hpp"
#include "VehiclePawnBase.h"
#include "FlyingPawn.generated.h"


UCLASS()
class AIRSIM_API AFlyingPawn : public AVehiclePawnBase
{
	GENERATED_BODY()

public: //interface
		//overrides from VehiclePawnBase
	virtual APIPCamera* getFpvCamera() override;
	virtual void initialize() override;

	msr::airlib::MavLinkDroneController::ConnectionInfo getMavConnectionInfo();
	void setRotorSpeed(int rotor_index, float radsPerSec);

public: //blueprint properties
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
		float RotatorFactor = 1.0f;

	//HIL settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MavLink")
		FString VehicleName = "Pixhawk";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MavLink")
		bool UseSerial = true;    //false = UDP
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MavLink")
		FString SerialPortName = "*";    // '*' means find port automatically, or use correct COM port like 'COM6' on Windows or /dev/ttyACM0 for linux
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MavLink")
		int SerialBaudrate = 115200;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MavLink")
		FString UdpIP = "127.0.0.1";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MavLink")
		int UdpPort = 14560;    // port that is publishing HIL_SENSOR messages, this needs to be routed to the actual drone.

private: //methods
	void setupComponentReferences();
	void setStencilIDs();

private: //variables
		 //Unreal components
	static constexpr size_t rotor_count = 4;
	UPROPERTY() APIPCamera* fpv_camera_;
	UPROPERTY() URotatingMovementComponent* rotating_movements_[rotor_count];
};

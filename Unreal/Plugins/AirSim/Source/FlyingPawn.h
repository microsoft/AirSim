#pragma once

#include <memory>
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

	void setRotorSpeed(int rotor_index, float radsPerSec);
    std::string getVehicleName();

public: //blueprint properties
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
		float RotatorFactor = 1.0f;

	//HIL settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HIL")
		FString VehicleName = "Pixhawk";

private: //methods
	void setupComponentReferences();
	void setStencilIDs();

private: //variables
		 //Unreal components
	static constexpr size_t rotor_count = 4;
	UPROPERTY() APIPCamera* fpv_camera_;
	UPROPERTY() URotatingMovementComponent* rotating_movements_[rotor_count];
};

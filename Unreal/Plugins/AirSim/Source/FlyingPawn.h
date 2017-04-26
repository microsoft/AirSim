#pragma once

#include <memory>
#include "VehiclePawnBase.h"
#include "controllers/DroneCommon.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include "FlyingPawn.generated.h"

UCLASS()
class AIRSIM_API AFlyingPawn : public AVehiclePawnBase
{
	GENERATED_BODY()

public: //interface
	void setRotorSpeed(int rotor_index, float radsPerSec);
    std::string getVehicleName();

public:
    //overrides from VehiclePawnBase
    virtual APIPCamera* getFpvCamera() override;
    virtual void initialize() override;
    virtual void reset() override;

public: //blueprint
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
		float RotatorFactor = 1.0f;

	//HIL settings
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HIL")
		FString VehicleName = "Pixhawk";

    UFUNCTION(BlueprintCallable, Category = "Init")
        void initializeForPlay();

private: //methods
	void setupComponentReferences();
	void setStencilIDs();
    void setupInputBindings();

private: //variables
		 //Unreal components
	static constexpr size_t rotor_count = 4;
	UPROPERTY() APIPCamera* fpv_camera_;
	UPROPERTY() URotatingMovementComponent* rotating_movements_[rotor_count];
};

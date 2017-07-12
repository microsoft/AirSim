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
    virtual APIPCamera* getCamera(int index = 0) override;
    virtual int getCameraCount() override;

    virtual void initialize() override;
    virtual void reset() override;

public: //blueprint
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
        float RotatorFactor = 1.0f;

    virtual void initializeForBeginPlay() override;

private: //methods
    void setupComponentReferences();
    void setStencilIDs();
    void setupInputBindings();

private: //variables
         //Unreal components
    static constexpr size_t rotor_count = 4;
    UPROPERTY() APIPCamera* fpv_camera_left_;
    UPROPERTY() APIPCamera* fpv_camera_right_;
    UPROPERTY() URotatingMovementComponent* rotating_movements_[rotor_count];
};

#include "SimModeWarthog.h"
#include "UObject/ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "WarthogPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/warthog/api/WarthogRpcLibServer.hpp"

extern CORE_API uint32 GFrameNumber;

void ASimModeWarthog::BeginPlay()
{
    Super::BeginPlay();

    initializePauseState();
}

void ASimModeWarthog::initializePauseState()
{
    pause_period_ = 0;
    pause_period_start_ = 0;
    pause(false);
}

void ASimModeWarthog::continueForTime(double seconds)
{
    pause_period_start_ = ClockFactory::get()->nowNanos();
    pause_period_ = seconds;
    pause(false);
}

void ASimModeWarthog::continueForFrames(uint32_t frames)
{
    targetFrameNumber_ = GFrameNumber + frames;
    frame_countdown_enabled_ = true;
    pause(false);
}

void ASimModeWarthog::setupClockSpeed()
{
    current_clockspeed_ = getSettings().clock_speed;

    //setup clock in PhysX
    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
    UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeWarthog::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    if (pause_period_start_ > 0) {
        if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
            if (!isPaused())
                pause(true);

            pause_period_start_ = 0;
        }
    }

    if (frame_countdown_enabled_) {
        if (targetFrameNumber_ <= GFrameNumber) {
            if (!isPaused())
                pause(true);

            frame_countdown_enabled_ = false;
        }
    }
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeWarthog::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::WarthogRpcLibServer(
        getApiProvider(), getSettings().api_server_address, getSettings().api_port));
#endif
}

void ASimModeWarthog::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeWarthog::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return (vehicle_type == AirSimSettings::kVehicleTypeWarthog);
}

std::string ASimModeWarthog::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == "")
        pawn_path = "DefaultWarthog";

    return pawn_path;
}

PawnEvents* ASimModeWarthog::getVehiclePawnEvents(APawn* pawn) const
{
    return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeWarthog::getVehiclePawnCameras(
    APawn* pawn) const
{
    return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}
void ASimModeWarthog::initializeVehiclePawn(APawn* pawn)
{
    auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
    vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);
}
std::unique_ptr<PawnSimApi> ASimModeWarthog::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn_sim_api_params.pawn);
    auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new WarthogPawnSimApi(pawn_sim_api_params,
                                                                         vehicle_pawn->getKeyBoardControls()));
    vehicle_sim_api->initialize();
    vehicle_sim_api->reset();
    return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeWarthog::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                        const PawnSimApi* sim_api) const
{
    const auto warthog_sim_api = static_cast<const WarthogPawnSimApi*>(sim_api);
    return warthog_sim_api->getVehicleApi();
}

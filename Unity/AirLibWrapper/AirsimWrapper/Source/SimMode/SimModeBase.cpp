#include "SimModeBase.h"
#include "common/ScalableClock.hpp"
#include "common/SteppableClock.hpp"
#include "common/EarthCelestial.hpp"
#include "../PInvokeWrapper.h"
#include "../WorldSimApi.h"

SimModeBase::SimModeBase(std::string vehicle_name, int port_number) :
	vehicle_name_(vehicle_name), port_number_(port_number)
{
}

void SimModeBase::BeginPlay()
{
	debug_reporter_.initialize(false);
	debug_reporter_.reset();
	global_ned_transform_.reset(new NedTransform(GetVehicleStartTransform()));
	world_sim_api_.reset(new WorldSimApi(this, vehicle_name_));
	api_provider_.reset(new msr::airlib::ApiProvider(world_sim_api_.get()));
	setupClockSpeed();
	record_tick_count = 0;
	setupVehiclesAndCamera();
}

void SimModeBase::Tick(float DeltaSeconds)
{
	getVehicleSimApi(vehicle_name_)->pawnTick(DeltaSeconds);
	showClockStats();
	updateDebugReport(debug_reporter_);
}

void SimModeBase::showClockStats()
{
	float clock_speed = getSettings().clock_speed;
	if (clock_speed != 1)
	{
		PrintLogMessage("ClockSpeed config, actual: ", 
			Utils::stringf("%f, %f", clock_speed, ClockFactory::get()->getTrueScaleWrtWallClock()).c_str(),
			vehicle_name_.c_str(), ErrorLogSeverity::Information);
	}
}

const NedTransform& SimModeBase::getGlobalNedTransform()
{
	return *global_ned_transform_;
}

void SimModeBase::EndPlay()
{
	world_sim_api_.reset();
	api_provider_.reset();
	api_server_.reset();
	global_ned_transform_.reset();
	vehicle_sim_apis_.clear();
}

bool SimModeBase::isPaused() const
{
	return false;
}

void SimModeBase::pause(bool is_paused)
{
	//should be overridden by derived class
	unused(is_paused);
	throw std::domain_error("Pause is not implemented by SimMode");
}

void SimModeBase::continueForTime(double seconds)
{
	//should be overridden by derived class
	unused(seconds);
	throw std::domain_error("continueForTime is not implemented by SimMode");
}

void SimModeBase::setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
    float celestial_clock_speed, float update_interval_secs, bool move_sun)
{
        unused(is_enabled);
        unused(start_datetime);
        unused(is_start_datetime_dst);
        unused(celestial_clock_speed);
        unused(update_interval_secs);
        unused(move_sun);
        //commenting this out for now to avoid unintentional Unity startup failure
        //throw std::domain_error("setTimeOfDay is not implemented by SimMode");
}

void SimModeBase::setWind(const msr::airlib::Vector3r& wind) const
{
    // should be overridden by derived class
    unused(wind);
    throw std::domain_error("setWind is not implemented by SimMode");
}

std::unique_ptr<msr::airlib::ApiServerBase> SimModeBase::createApiServer() const
{
	//this will be the case when compilation with RPCLIB is disabled or simmode doesn't support APIs
	return nullptr;
}

void SimModeBase::setupClockSpeed()
{
	//default setup - this should be overridden in derived modes as needed

	float clock_speed = getSettings().clock_speed;

	//setup clock in ClockFactory
	std::string clock_type = getSettings().clock_type;

	if (clock_type == "ScalableClock")
		ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
	else if (clock_type == "SteppableClock")
		ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
			static_cast<msr::airlib::TTimeDelta>(msr::airlib::SteppableClock::DefaultStepSize * clock_speed)));
	else
		throw std::invalid_argument(common_utils::Utils::stringf(
			"clock_type %s is not recognized", clock_type.c_str()));
}

void SimModeBase::reset()
{
	for (auto& api : getApiProvider()->getVehicleSimApis()) {
		api->reset();
	}
}

std::string SimModeBase::getDebugReport()
{
	return debug_reporter_.getOutput();
}

const msr::airlib::AirSimSettings& SimModeBase::getSettings() const
{
	return AirSimSettings::singleton();
}

//API server start/stop
void SimModeBase::startApiServer()
{
	if (getSettings().enable_rpc) {

#ifdef AIRLIB_NO_RPC
		api_server_.reset();
#else
		api_server_ = createApiServer();
#endif
		try {
			api_server_->start(false, 4); //TODO: set thread for vehicle count
		}
		catch (std::exception& ex) {
			PrintLogMessage("Cannot start RpcLib Server", ex.what(), vehicle_name_.c_str(), ErrorLogSeverity::Error);
		}
	}
	else
		PrintLogMessage("API server is disabled in settings", "", vehicle_name_.c_str(), ErrorLogSeverity::Information);

}
void SimModeBase::stopApiServer()
{
	if (api_server_ != nullptr) {
		api_server_->stop();
		api_server_.reset(nullptr);
	}
}

bool SimModeBase::isApiServerStarted()
{
	return api_server_ != nullptr;
}

void SimModeBase::updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter)
{
	debug_reporter.update();
	debug_reporter.setEnable(EnableReport);

	if (debug_reporter.canReport()) {
		debug_reporter.clearReport();

		for (auto& api : getApiProvider()->getVehicleSimApis()) {
			PawnSimApi* vehicle_sim_api = static_cast<PawnSimApi*>(api);
			msr::airlib::StateReporter& reporter = *debug_reporter.getReporter();
			std::string vehicle_name = vehicle_sim_api->getVehicleName();

			reporter.writeHeading(std::string("Vehicle: ").append(
				vehicle_name == "" ? "(default)" : vehicle_name));

			const msr::airlib::Kinematics::State* kinematics = vehicle_sim_api->getGroundTruthKinematics();

			reporter.writeValue("Position", kinematics->pose.position);
			reporter.writeValue("Orientation", kinematics->pose.orientation);
			reporter.writeValue("Lin-Vel", kinematics->twist.linear);
			reporter.writeValue("Lin-Accl", kinematics->accelerations.linear);
			reporter.writeValue("Ang-Vel", kinematics->twist.angular);
			reporter.writeValue("Ang-Accl", kinematics->accelerations.angular);
		}
	}
}

void SimModeBase::setupVehiclesAndCamera()
{
	//determine camera director camera default pose and spawn it
	const auto& camera_director_setting = getSettings().camera_director;
	UnityPawn* vehicle_pawn = GetVehiclePawn();
	const auto& home_geopoint = msr::airlib::EarthUtils::nedToGeodetic(GetVehiclePosition(), getSettings().origin_geopoint);

	PawnSimApi::Params pawn_sim_api_params(vehicle_pawn, &getGlobalNedTransform(), home_geopoint, vehicle_name_);
	auto vehicle_sim_api = createVehicleSimApi(pawn_sim_api_params);
	auto vehicle_sim_api_p = vehicle_sim_api.get();
	auto vehicle_Api = getVehicleApi(pawn_sim_api_params, vehicle_sim_api_p);

	getApiProvider()->insert_or_assign(vehicle_name_, vehicle_Api, vehicle_sim_api_p);

	if ((!getApiProvider()->hasDefaultVehicle()) && vehicle_name_ != "")
	{
		getApiProvider()->makeDefaultVehicle(vehicle_name_);
	}
	vehicle_sim_apis_.push_back(std::move(vehicle_sim_api));
}

const msr::airlib::Vector3r SimModeBase::GetVehiclePosition()
{
	AirSimPose airSimPose = GetPose(vehicle_name_.c_str());
	msr::airlib::Vector3r vehiclePosition(airSimPose.position.x, airSimPose.position.y, airSimPose.position.z);
	return vehiclePosition;
}

UnityPawn* SimModeBase::GetVehiclePawn()
{
	return nullptr;
}

bool SimModeBase::isVehicleTypeSupported(const std::string& vehicle_type) const
{
	//derived class should override this method to retrieve types of pawns they support
	return false;
}

std::unique_ptr<PawnSimApi> SimModeBase::createVehicleSimApi(
	const PawnSimApi::Params& pawn_sim_api_params) const
{
	unused(pawn_sim_api_params);
    auto sim_api = std::unique_ptr<PawnSimApi>();
    sim_api->initialize();
    return sim_api;
}

msr::airlib::VehicleApiBase* SimModeBase::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
	const PawnSimApi* sim_api) const
{
	//derived class should override this method to retrieve types of pawns they support
	return nullptr;
}

UnityTransform SimModeBase::GetVehicleStartTransform()
{
	UnityTransform unityTransform = GetTransformFromUnity(vehicle_name_.c_str());
	return unityTransform;
}
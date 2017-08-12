// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AutomobileSedanController_hpp
#define msr_airlib_AutomobileSedanController_hpp

#include <exception>
#include "controllers/ControllerBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "automobile/AutomobileSedan.hpp"
#include "controllers/VehicleCamera.hpp"

namespace msr { namespace airlib {

	class AutomobileSedanController : public ControllerBase {
		private:
			AutomobileSedan* vehicle_;

			// Control variables
			real_T steeringAngle_;
			real_T throttlePercentage_;
			real_T brakePercentage_;

		public:

			unordered_map<int, std::shared_ptr<VehicleCamera>> enabled_cameras;
			AutomobileSedanController(AutomobileSedan* vehicle)
			{
				vehicle_ = vehicle;
				reset();
			}

			//*** Start ControllerBase implementation ****//
			virtual void reset() override
			{
				steeringAngle_ = static_cast<real_T>(0);
				throttlePercentage_ = static_cast<real_T>(0);
				brakePercentage_ = static_cast<real_T>(0);
			}

			virtual void update() override
			{
			}

			virtual real_T getVertexControlSignal(unsigned int rotor_index) override
			{
				return static_cast<real_T>(0);
			}
			virtual size_t getVertexCount() override
			{
				return 0;
			}
			//*** End ControllerBase implementation ****//

			virtual void setVehicleControlSignals(real_T steeringAngle, real_T throttlePercentage, real_T brakePercentage)
			{
				steeringAngle_ = steeringAngle;
				throttlePercentage_ = throttlePercentage;
				brakePercentage_ = brakePercentage;

				if (vehicle_ != nullptr)
				{
					vehicle_->SetControlSignals(steeringAngle_, throttlePercentage_, brakePercentage_);
				}
			}

			virtual real_T getSteeringAngle() const
			{
				return steeringAngle_;
			}

			virtual real_T getThrottlePercentage() const
			{
				return throttlePercentage_;
			}

			virtual real_T getBrakePercentage() const
			{
				return brakePercentage_;
			}

			vector<uint8_t> AutomobileSedanController::getImageFromCamera(int camera_id, VehicleCamera::ImageType type)
			{
				//StatusLock lock(this);
				vector<uint8_t> png;

				//TODO: perf work
				auto it = enabled_cameras.find(camera_id);
				if (it != enabled_cameras.end()) {
					it->second->getScreenShot(type, png);
				}
				return png;
			}

			void AutomobileSedanController::addCamera(std::shared_ptr<VehicleCamera> camera)
			{
				//StatusLock lock(this);
				enabled_cameras[camera->getId()] = camera;
			}

			virtual ~AutomobileSedanController() = default;

	};

}} //namespace
#endif

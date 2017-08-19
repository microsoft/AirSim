// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_AutomobileSedanController_hpp
#define msr_airlib_AutomobileSedanController_hpp

#include <exception>
#include "controllers/ControllerBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "automobile/AutomobileSedan.hpp"
#include "controllers/VehicleCameraBase.hpp"

namespace msr { namespace airlib {

	class AutomobileSedanController : public ControllerBase {
		private:
			AutomobileSedan* vehicle_;
			vector<VehicleCameraBase*> cameras_;

			// Control variables
			real_T steeringAngle_;
			real_T throttlePercentage_;
			real_T brakePercentage_;

		public:
			struct ImageRequest {
				uint8_t camera_id;
				VehicleCameraBase::ImageType image_type;
				bool pixels_as_float;
				bool compress;

				ImageRequest()
				{}

				ImageRequest(uint8_t camera_id_val, VehicleCameraBase::ImageType image_type_val, bool pixels_as_float_val = false, bool compress_val = true)
				{
					camera_id = camera_id_val;
					image_type = image_type_val;
					pixels_as_float = pixels_as_float_val;
					compress = compress_val;
				}
			};

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
				return static_cast<real_T>(rotor_index);
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

			void simAddCamera(VehicleCameraBase* camera)
			{
				cameras_.push_back(camera);
			}

			VehicleCameraBase* simGetCamera(int index)
			{
				return cameras_.at(index);
			}

			vector<VehicleCameraBase::ImageResponse> simGetImages(const vector<AutomobileSedanController::ImageRequest>& request)
			{
				vector<VehicleCameraBase::ImageResponse> response;

				for (const auto& item : request) {
					VehicleCameraBase* camera = simGetCamera(item.camera_id);
					const auto& item_response = camera->getImage(item.image_type, item.pixels_as_float, item.compress);
					response.push_back(item_response);
				}

				return response;
			}

			vector<uint8_t> simGetImage(uint8_t camera_id, VehicleCameraBase::ImageType image_type)
			{
				vector<AutomobileSedanController::ImageRequest> request = { AutomobileSedanController::ImageRequest(camera_id, image_type) };
				const vector<VehicleCameraBase::ImageResponse>& response = simGetImages(request);
				if (response.size() > 0)
					return response.at(0).image_data;
				else
					return vector<uint8_t>();
			}

			virtual ~AutomobileSedanController() = default;

	};

}} //namespace
#endif

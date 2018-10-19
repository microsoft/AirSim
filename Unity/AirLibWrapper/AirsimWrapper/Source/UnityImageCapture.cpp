#include "UnityImageCapture.h"
#include "PInvokeWrapper.h"
#include "UnityUtilities.hpp"

namespace AirSimUnity
{
	UnityImageCapture::UnityImageCapture(std::string vehicle_name) : vehicle_name_(vehicle_name)
	{}

	UnityImageCapture::~UnityImageCapture()
	{}

	// implements getImages() method in the ImageCaptureBase class.
	void UnityImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
		std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
	{
		if (requests.size() > 0)
		{
			for (int i = 0; i < requests.size(); i++)
			{
				ImageResponse airsim_response;
				responses.push_back(airsim_response);
				AirSimImageRequest request = UnityUtilities::Convert_to_UnityRequest(requests[i]);
				AirSimImageResponse response = GetSimImages(request, vehicle_name_.c_str());  //Into Unity
				UnityUtilities::Convert_to_AirsimResponse(response, responses[i], request.camera_name);
			}
		}
	}
}
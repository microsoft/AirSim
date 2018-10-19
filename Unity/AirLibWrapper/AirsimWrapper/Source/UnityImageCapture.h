#pragma once

#include "common/ImageCaptureBase.hpp"
#include "AirSimStructs.hpp"

using namespace AirSimUnity;

/*
* Implementation of ImageCaptureBase(defined in AirLib) that is used  by vehicles to capture images.
*/

namespace AirSimUnity
{
	class UnityImageCapture : public msr::airlib::ImageCaptureBase
	{
	public:
		typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
		UnityImageCapture(std::string vehicle_name);
		virtual ~UnityImageCapture();
		virtual void getImages(const std::vector<ImageRequest>& requests, std::vector<ImageResponse>& responses) const;

	private:
		std::string vehicle_name_;
	};
}
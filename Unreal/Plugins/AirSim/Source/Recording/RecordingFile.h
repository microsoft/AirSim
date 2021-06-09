#pragma once

#include "CoreMinimal.h"
#include <string>
#include "AirBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "HAL/FileManager.h"
#include "PawnSimApi.h"

class RecordingFile
{
public:
    ~RecordingFile();

    void appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, msr::airlib::VehicleSimApiBase* vehicle_sim_api) const;
    void appendColumnHeader(const std::string& header_columns);
    void startRecording(msr::airlib::VehicleSimApiBase* vehicle_sim_api, const std::string& folder = "");
    void stopRecording(bool ignore_if_stopped);
    bool isRecording() const;

private:
    void createFile(const std::string& file_path, const std::string& header_columns);
    void closeFile();
    void writeString(const std::string& line) const;
    bool isFileOpen() const;

private:
    std::string record_filename = "airsim_rec";
    std::string image_path_;
    bool is_recording_ = false;
    IFileHandle* log_file_handle_ = nullptr;
};
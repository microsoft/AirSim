#pragma once

#include "CoreMinimal.h"
#include <string>
#include "AirBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "FileManager.h"
#include "PawnSimApi.h"


class RecordingFile {
public:
    ~RecordingFile();

    void appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, msr::airlib::VehicleSimApiBase* vehicle_sim_api);
    void appendColumnHeader(const std::string& header_columns);
    void startRecording(msr::airlib::VehicleSimApiBase* vehicle_sim_api);
    void stopRecording(bool ignore_if_stopped);
    bool isRecording();

private:
    void createFile(const std::string& file_path, const std::string& header_columns);
    void closeFile();
    void writeString(const std::string& line);
    bool isFileOpen();

private:
    std::string record_filename = "airsim_rec";     
    unsigned int images_saved_ = 0;
    std::string image_path_;
    bool is_recording_ = false;
    IFileHandle* log_file_handle_ = nullptr;
};
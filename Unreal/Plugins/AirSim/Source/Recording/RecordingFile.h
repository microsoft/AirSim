#pragma once

#include "CoreMinimal.h"
#include <string>
#include "AirBlueprintLib.h"
#include "physics/Kinematics.hpp"
#include "FileManager.h"

//TODO: move struct to its own file?
struct RecordingSettings {
    bool record_on_move = false;
    float record_interval = 0.05f;
};

class RecordingFile {
public:
    ~RecordingFile();

    void appendRecord(TArray<uint8>& compressedPng, const msr::airlib::Kinematics* kinematics);
    void startRecording();
    void stopRecording();
    bool isRecording();

private:
    void createFile(const std::string& file_path);
    void closeFile();
    void writeString(const std::string& line);
    bool isFileOpen();
    std::string getLogFileFullPath();
    std::string getLine(const msr::airlib::Kinematics::State& kinematics);


private:
    std::string record_filename = "airsim_rec";     
    unsigned int images_saved_ = 0;
    FString image_path_;
    bool is_recording_ = false;
    IFileHandle* log_file_handle_ = nullptr;
};
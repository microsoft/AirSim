#pragma once

#include <string>
#include "AirBlueprintLib.h"
#include "physics/PhysicsBody.hpp"
#include "FileManager.h"

//TODO: move struct to its own file?
struct RecordingSettings {
    bool record_on_move = true;
    float record_interval = 0.05f;
};

class RecordingFile {
public:
    RecordingFile();
    ~RecordingFile();

    void appendRecord(TArray<uint8>& compressedPng, const msr::airlib::PhysicsBody* physics_body);
    void startRecording();
    void stopRecording();
    bool isRecording();
    void initializeForPlay();

private:
    void createFile(const std::string& file_path);
    void closeFile();
    void writeLine(const std::string& line);
    bool isFileOpen();


private:
    std::string record_filename = "airsim_rec";
    unsigned int images_saved_ = 0;
    FString image_path_;
    bool is_recording_;

    IFileHandle* log_file_handle_;
};
#include "AirBlueprintLib.h"
#include "physics/PhysicsBody.hpp"


class RecordingFile {
public:
    void appendRecord(FString image_path, TArray<uint8>& compressedPng, const msr::airlib::PhysicsBody* physics_body);
    void startRecording();
    void stopRecording();
    bool isRecording();
    void initializeForPlay();

private:
    std::ofstream record_file;
    std::string record_filename = "airsim_rec";
    unsigned int images_saved_ = 0;
    bool is_recording_;
};
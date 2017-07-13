#include <AirSim.h>
#include "ImageUtils.h"
#include "common/ClockFactory.hpp"
#include "common/common_utils/FileSystem.hpp"

void RecordingFile::appendRecord(FString image_path, TArray<uint8>& compressedPng, const msr::airlib::PhysicsBody* physics_body)
{
    if (compressedPng.Num() == 0)
        return;

    FString filePath = image_path + FString::FromInt(images_saved_) + ".png";
    bool imageSavedOk = FFileHelper::SaveArrayToFile(compressedPng, *filePath);

    // If render command is complete, save image along with position and orientation

    if (!imageSavedOk)
        UAirBlueprintLib::LogMessage(TEXT("FAILED to save screenshot to:"), filePath, LogDebugLevel::Failure);
    else {
        auto kinematics = physics_body->getKinematics();

        uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

        record_file << timestamp_millis << "\t";
        record_file << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
        record_file << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
        record_file << "\n";

        UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);
        images_saved_++;
    }
}

void RecordingFile::startRecording()
{
    if (record_file.is_open()) {
        record_file.close();
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was already open"), LogDebugLevel::Failure);
    }

    std::string fullPath = common_utils::FileSystem::getLogFileNamePath(record_filename, "", ".txt", true);
    common_utils::FileSystem::createTextFile(fullPath, record_file);

    if (record_file.is_open()) {
        is_recording_ = true;

        UAirBlueprintLib::LogMessage(TEXT("Recording"), TEXT("Started"), LogDebugLevel::Success);
    }
    else
        UAirBlueprintLib::LogMessage("Error creating log file", fullPath.c_str(), LogDebugLevel::Failure);
}

void RecordingFile::stopRecording()
{
    is_recording_ = false;
    if (!record_file.is_open()) {
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was not open"), LogDebugLevel::Failure);
    }
    else
        record_file.close();

    UAirBlueprintLib::LogMessage(TEXT("Recording"), TEXT("Stopped"), LogDebugLevel::Success);
}

bool RecordingFile::isRecording()
{
    return is_recording_;
}

void RecordingFile::initializeForPlay()
{
    is_recording_ = false;
}
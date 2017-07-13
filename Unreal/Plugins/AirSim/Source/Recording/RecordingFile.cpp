#include <AirSim.h>
#include <sstream>
#include "ImageUtils.h"
#include "common/ClockFactory.hpp"
#include "common/common_utils/FileSystem.hpp"

RecordingFile::RecordingFile()
    : image_path_(FString(common_utils::FileSystem::getLogFileNamePath("img_", "", "", false).c_str())),
    log_file_handle_(nullptr)
{
}

void RecordingFile::appendRecord(TArray<uint8>& image_data, const msr::airlib::PhysicsBody* physics_body)
{
    if (image_data.Num() == 0)
        return;

    FString filePath = image_path_ + FString::FromInt(images_saved_) + ".png";
    bool imageSavedOk = FFileHelper::SaveArrayToFile(image_data, *filePath);

    // If render command is complete, save image along with position and orientation

    if (!imageSavedOk)
        UAirBlueprintLib::LogMessage(TEXT("FAILED to save screenshot to:"), filePath, LogDebugLevel::Failure);
    else {
        auto kinematics = physics_body->getKinematics();

        uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

        std::stringstream ss;
        ss << timestamp_millis << "\t";
        ss << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
        ss << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
        ss << "\n";

        writeLine(ss.str());

        UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);
        images_saved_++;
    }
}

void RecordingFile::createFile(const std::string& file_path)
{
    closeFile();

    IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
    log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));
}

bool RecordingFile::isFileOpen()
{
    return log_file_handle_ != nullptr;
}

void RecordingFile::closeFile()
{
    if (isFileOpen())
        delete log_file_handle_;

    log_file_handle_ = nullptr;
}

void RecordingFile::writeLine(const std::string& line)
{

}

RecordingFile::~RecordingFile()
{
    closeFile();
}

void RecordingFile::startRecording()
{
    std::string fullPath = common_utils::FileSystem::getLogFileNamePath(record_filename, "", ".txt", true);
    createFile(fullPath);

    if (isFileOpen()) {
        is_recording_ = true;

        UAirBlueprintLib::LogMessage(TEXT("Recording"), TEXT("Started"), LogDebugLevel::Success);
    }
    else
        UAirBlueprintLib::LogMessage("Error creating log file", fullPath.c_str(), LogDebugLevel::Failure);
}

void RecordingFile::stopRecording()
{
    is_recording_ = false;
    if (isFileOpen()) {
        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was not open"), LogDebugLevel::Failure);
    }
    else
        closeFile();

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
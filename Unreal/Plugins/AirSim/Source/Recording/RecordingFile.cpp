#include "RecordingFile.h"
#include "HAL/PlatformFilemanager.h"
#include "FileHelper.h"
#include <sstream>
#include "ImageUtils.h"
#include "common/ClockFactory.hpp"
#include "common/common_utils/FileSystem.hpp"


RecordingFile::RecordingFile(const std::vector <std::string>& columns)
{
    this->columns_ = columns;
}
void RecordingFile::appendRecord(const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, VehiclePawnWrapper* wrapper)
{   bool save_success = false;
    std::stringstream image_file_names;

    for (auto i = 0; i < responses.size(); ++i) {
        const auto& response = responses.at(i);

        std::stringstream image_file_name;
        image_file_name << "img_" << response.camera_id << "_" <<
            common_utils::Utils::toNumeric(response.image_type) << "_" <<
            common_utils::Utils::getTimeSinceEpochNanos() << 
            (response.pixels_as_float ? ".pfm" : ".png");

        if (i > 0)
            image_file_names << ";";
        image_file_names << image_file_name.str();

        std::string image_full_file_path = common_utils::FileSystem::combine(image_path_, image_file_name.str());

        try {
            if (response.pixels_as_float) {
                common_utils::Utils::writePfmFile(response.image_data_float.data(), response.width, response.height,
                    image_full_file_path);
            }
            else {
                std::ofstream file(image_full_file_path, std::ios::binary);
                file.write(reinterpret_cast<const char*>(response.image_data_uint8.data()), response.image_data_uint8.size());
                file.close();
            }

            save_success = true;
        }
        catch(std::exception& ex) {
            save_success = false;
            UAirBlueprintLib::LogMessage(TEXT("Image file save failed"), FString(ex.what()), LogDebugLevel::Failure);        
        }
    }

    if (save_success) {
        writeString(wrapper->getLogLine().append(image_file_names.str()).append("\n"));

        //UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);
        images_saved_++;
    }
}

void RecordingFile::appendColumnHeader(const std::vector<std::string>& columns)
{
    std::string line;
    for (int i = 0; i < columns.size()-1; i++) 
    {
        line.append(columns[i]).append("\t");
    }
    line.append(columns[columns.size() - 1]).append("\n");

    writeString(line);
}

void RecordingFile::createFile(const std::string& file_path)
{
    try {
        closeFile();

        IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
        log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));
        appendColumnHeader(this->columns_);
    }
    catch(std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("createFile Failed for ") + file_path, ex.what(), LogDebugLevel::Failure);        
    }
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

void RecordingFile::writeString(const std::string& str)
{
    try {    
        if (log_file_handle_) {
            FString line_f = FString(str.c_str());
            log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());
        }
        else
            UAirBlueprintLib::LogMessageString("Attempt to write to recording log file when file was not opened", "", LogDebugLevel::Failure);
    }
    catch(std::exception& ex) {
        UAirBlueprintLib::LogMessageString(std::string("file write to recording file failed "), ex.what(), LogDebugLevel::Failure);        
    }
}

RecordingFile::~RecordingFile()
{
    stopRecording(true);
}

void RecordingFile::startRecording()
{
    try {
        std::string log_folderpath = common_utils::FileSystem::getLogFolderPath(true);
        image_path_ = common_utils::FileSystem::ensureFolder(log_folderpath, "images");
        std::string log_filepath = common_utils::FileSystem::getLogFileNamePath(log_folderpath, record_filename, "", ".txt", false);
        if (log_filepath != "")
            createFile(log_filepath);
        else {
            UAirBlueprintLib::LogMessageString("Cannot start recording because path for log file is not available", "", LogDebugLevel::Failure);
            return;
        }

        if (isFileOpen()) {
            is_recording_ = true;

            UAirBlueprintLib::LogMessage(TEXT("Recording: "), TEXT("Started"), LogDebugLevel::Success);
        }
        else
            UAirBlueprintLib::LogMessageString("Error creating log file", log_filepath.c_str(), LogDebugLevel::Failure);
    }
    catch(...) {
        UAirBlueprintLib::LogMessageString("Error in startRecording", "", LogDebugLevel::Failure);
    }
}

void RecordingFile::stopRecording(bool ignore_if_stopped)
{
    is_recording_ = false;
    if (! isFileOpen()) {
        if (ignore_if_stopped)
            return;

        UAirBlueprintLib::LogMessage(TEXT("Recording Error"), TEXT("File was not open"), LogDebugLevel::Failure);
    }
    else
        closeFile();

    UAirBlueprintLib::LogMessage(TEXT("Recording: "), TEXT("Stopped"), LogDebugLevel::Success);
    UAirBlueprintLib::LogMessage(TEXT("Data saved to: "), FString(image_path_.c_str()), LogDebugLevel::Success);
}

bool RecordingFile::isRecording()
{
    return is_recording_;
}

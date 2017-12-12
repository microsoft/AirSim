#pragma once

struct RecordingSettings {
    bool record_on_move = false;
    float record_interval = 0.05f;

    std::vector<std::string> header_columns;
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;
};
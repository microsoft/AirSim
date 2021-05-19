// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#pragma once

#include "../sgmstereo/sgmstereo.h"
#include "SGMOptions.h"
#include <vector>

class CStateStereo
{
private:
    float downSampleFactor;
    std::string outputDir;
    int inputFrameWidth, inputFrameHeight;
    int ndisps;
    int minDisp;
    int maxDisp;
    int confThreshold;

    SGMStereo* sgmStereo;

public:
    int processingFrameWidth, processingFrameHeight;

    CStateStereo();
    ~CStateStereo();
    void Initialize(SGMOptions& params, int m = 144, int n = 256);
    void CleanUp();
    void ProcessFrameAirSim(int frameCounter, float& dtime, const std::vector<uint8_t>& left_image, const std::vector<uint8_t>& right_image);
    float GetLeftDisparity(float x, float y);

    float* dispMap;
    unsigned char* confMap;
};
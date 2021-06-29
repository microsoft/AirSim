// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#include "StateStereo.h"
#include "sgmstereo.h"
#include <stdio.h> /* printf */
#include <ctime> /* clock_t, clock, CLOCKS_PER_SEC */

CStateStereo::CStateStereo()
{
}

CStateStereo::~CStateStereo()
{
}

void CStateStereo::Initialize(SGMOptions& params, int m, int n)
{
    inputFrameWidth = n;
    inputFrameHeight = m;

    outputDir = params.outputDir;
    minDisp = params.minDisparity;
    maxDisp = params.maxDisparity;
    ndisps = maxDisp - minDisp;
    confThreshold = params.sgmConfidenceThreshold;

    // ensure that disparity range is a multiple of 8
    int rem = ndisps % 8;
    int dv = ndisps / 8;
    if (rem != 0) {
        ndisps = 8 * (dv + 1);
        maxDisp = ndisps + minDisp;
    }

    if (params.maxImageDimensionWidth == -1 || inputFrameWidth <= params.maxImageDimensionWidth) {
        downSampleFactor = 1.0f;
        processingFrameWidth = inputFrameWidth;
        processingFrameHeight = inputFrameHeight;
    }
    else {
        downSampleFactor = params.maxImageDimensionWidth / (1.0f * inputFrameWidth);
        processingFrameWidth = params.maxImageDimensionWidth;
        processingFrameHeight = (int)(downSampleFactor * inputFrameHeight + 0.5f);
    }

    printf("process at %d x %d resolution, ndisps = %d\n", processingFrameWidth, processingFrameHeight, ndisps);

    dispMap = new float[processingFrameWidth * processingFrameHeight];
    confMap = new unsigned char[processingFrameWidth * processingFrameHeight];

    sgmStereo = new SGMStereo(processingFrameWidth, processingFrameHeight, -maxDisp, -minDisp, params.numDirections, params.sgmConfidenceThreshold, params.doSubPixRefinement, params.smoothness, params.penalty1, params.penalty2, params.alpha, params.doSequential);
}

void CStateStereo::CleanUp()
{
    if (sgmStereo != NULL) {
        sgmStereo->free();
    }
}

void CStateStereo::ProcessFrameAirSim(int frameCounter, float& dtime, const std::vector<uint8_t>& left_image, const std::vector<uint8_t>& right_image)
{
    unsigned char *iL, *iR;

    // sgm stereo
    if (processingFrameWidth != inputFrameWidth || processingFrameHeight != inputFrameHeight) {
        printf("[ERROR]: Frame resolution = (%d x %d) is not equal to initialization ...\n", processingFrameWidth, processingFrameHeight);
    }

    int nP = processingFrameWidth * processingFrameHeight;
    iL = new unsigned char[nP];
    iR = new unsigned char[nP];

    int channels = (int)left_image.size() / nP;

    for (int i = 0; i < nP; i++) {
        {
            int idx = channels * i;
            iL[i] = (left_image[idx] + left_image[idx + 1] + left_image[idx + 2]) / 3;
            iR[i] = (right_image[idx] + right_image[idx + 1] + right_image[idx + 2]) / 3;
        }
    }

    std::clock_t start;
    start = std::clock();

    sgmStereo->Run(iL, iR, dispMap, confMap);
    float duration = (std::clock() - start) / (float)CLOCKS_PER_SEC;
    dtime += duration;

    printf("Frame %06d:	%5.1f ms, Average fps: %lf\n", frameCounter, duration * 1000, 1.0 / (dtime / double(frameCounter + 1)));

    delete[] iL;
    delete[] iR;
}

float CStateStereo::GetLeftDisparity(float x, float y)
{
    int ix = (int)(x * processingFrameWidth + 0.5f);
    int iy = (int)(y * processingFrameHeight + 0.5f);
    ix = __max(ix, 0);
    ix = __min(ix, processingFrameWidth - 1);
    iy = __max(iy, 0);
    iy = __min(iy, processingFrameHeight - 1);
    int off = iy * processingFrameWidth + ix;
    float d = dispMap[off];
    unsigned char c = confMap[off];
    if (fabs(d) < ndisps && c >= confThreshold) {
        return 1.0f - (float)(fabs(d) / ndisps);
    }
    else
        return -1.0f;
}
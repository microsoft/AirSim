// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#pragma once

#include <stdio.h> /* printf */
#include <string>

struct SGMOptions
{
    std::string inputDir; // folder contains input images
    std::string outputDir; // folder for output features files and match files
    int onlyStereo; // only stereo processing

    int sgmConfidenceThreshold; // value between [0 and 255] confidence threshold cutoff
    int maxImageDimensionWidth;
    int minDisparity;
    int maxDisparity;
    int numDirections; // 4 or 8
    int doSequential; // do sequential message passing (or horizontal and vertical in parallel).
    int doVis; // if 1, then output visualization.
    int doOut; // if 1, then write output
    float smoothness;
    float penalty1;
    float penalty2;
    float alpha;
    int doSubPixRefinement;

    SGMOptions()
    {
        inputDir = "";
        outputDir = "C:/Github/AirSimSGM/SGM/output";
        sgmConfidenceThreshold = 16;
        maxImageDimensionWidth = -1;
        minDisparity = 0;
        maxDisparity = 192;
        numDirections = 4;
        doSequential = 0;
        doVis = 1;
        doOut = 0;
        smoothness = 200.0f;
        penalty1 = 1.0;
        penalty2 = 8.0;
        alpha = 10.0;
        onlyStereo = 0;
        doSubPixRefinement = 1;
    }

    void Print()
    {
        printf("\n\n****************** Parameter List *********************\n");
        wprintf(L"   sgmConfidenceThreshold = %d\n", sgmConfidenceThreshold);
        wprintf(L"   maxImageDimensionWidth = %d\n", maxImageDimensionWidth);
        wprintf(L"   minDisparity = %d\n", minDisparity);
        wprintf(L"   maxDisparity = %d\n", maxDisparity);
        wprintf(L"   numDirections = %d\n", numDirections);
        wprintf(L"   smoothness = %f\n", smoothness);
        wprintf(L"   penalty1 = %f\n", penalty1);
        wprintf(L"   penalty2 = %f\n", penalty2);
        wprintf(L"   alpha = %f\n", alpha);
        wprintf(L"   doSequential = %d\n", doSequential);
        wprintf(L"   doVis = %d\n", doVis);
        wprintf(L"   doOut = %d\n", doOut);
        wprintf(L"   onlyStereo = %d\n", onlyStereo);
        wprintf(L"   doSubPixRefinement = %d\n", doSubPixRefinement);
        printf("*********************************************************\n\n\n");
    }
};
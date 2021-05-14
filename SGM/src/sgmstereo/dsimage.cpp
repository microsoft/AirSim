// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#include "dsimage.h"

void getDispMap2(DSI& dv1, DSI& dv2, int confThreshold, float* dispMap, unsigned char* confMap)
{
    int cols = (int)dv1.m_cols;
    int rows = (int)dv1.m_rows;
    int planes = (int)dv1.m_planes;

    // first row
    {
        uint64_t offset = 0;
        float* pDisp = &(dispMap[offset]);
        unsigned char* pConf = &(confMap[offset]);
        for (int x = 0; x < cols; x++) {
            pDisp[x] = FLT_MAX;
            pConf[x] = 0;
        }
    }

    // last row
    {
        uint64_t offset = (rows - 1) * cols;
        float* pDisp = &(dispMap[offset]);
        unsigned char* pConf = &(confMap[offset]);
        for (int x = 0; x < cols; x++) {
            pDisp[x] = FLT_MAX;
            pConf[x] = 0;
        }
    }

    for (int y = 0; y < rows; y++) {
        uint64_t offset = y * cols;
        float* pDisp = &(dispMap[offset]);
        unsigned char* pConf = &(confMap[offset]);
        {
            pDisp[0] = FLT_MAX;
            pConf[0] = 0;
            pDisp[cols - 1] = FLT_MAX;
            pConf[cols - 1] = 0;
        }
    }

    for (int y = 1; y < rows - 1; y++) {
        uint64_t offset = y * cols;
        float* pDisp = &(dispMap[offset]);
        unsigned char* pConf = &(confMap[offset]);

#pragma omp parallel for schedule(dynamic, 1)

        for (int x = 1; x < cols - 1; x++) {
            int bestplane = (int)planes - 1;
            short minval = SHRT_MAX;
            short secondminval = SHRT_MAX;
            short* pV1 = dv1(x, y);
            short* pV2 = dv2(x, y);
            for (int d = 0; d < planes; d++) {
                short val = pV1[d] + pV2[d];
                if (val < minval) {
                    minval = val;
                    bestplane = d;
                }
            }

            for (int d = 0; d < planes; d++) {
                if (abs(d - bestplane) > 2) {
                    short val = pV1[d] + pV2[d];
                    if (val < secondminval) {
                        secondminval = val;
                    }
                }
            }

            float distinctiveness1 = float(minval) / float(secondminval + 1e-9f);
            float conf = (float)__min(__max(20.0f * log(1.0f / (distinctiveness1 * distinctiveness1)), 0.0f), 255.0f);
            int Dim = (int)planes;
            if (conf >= confThreshold) {
                // Local quadratic fit of cost and subpixel refinement.
                double rDisp = bestplane;
                double rCost = minval;
                if (bestplane >= 1 && bestplane < planes - 1) {
                    double yl = pV1[bestplane - 1] + pV2[bestplane - 1];
                    double xc = bestplane;
                    double yc = minval;
                    double yu = pV1[bestplane + 1] + pV2[bestplane + 1];
                    double d2 = yu - yc + yl - yc;
                    double d1 = 0.5 * (yu - yl);
                    if (fabs(d2) > fabs(d1)) {
                        rDisp = xc - d1 / d2;
                        rCost = yc + 0.5 * d1 * (rDisp - xc);
                    }
                }
                pDisp[x] = (float)(rDisp - Dim);
                pConf[x] = (unsigned char)conf;
            }
            else {
                pDisp[x] = FLT_MAX;
                pConf[x] = 0;
            }
        }
    }
}
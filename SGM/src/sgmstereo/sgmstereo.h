// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#ifndef sgm_stereo_h
#define sgm_stereo_h

#include "dsimage.h"

class SGMStereo
{
private:
    void calculateDSI_sse(unsigned char* refImage, unsigned char* nbrImage);
    void messagePassing(short* pData, short* pBuffer1, short* pDMessage, int size, float weight, short smoothness);
    void scanlineOptimization(DSI& dv, DSI& messages, unsigned char* img, float* lut, int dx_, int dy_);
    void scanlineOptimization_hor(DSI& dv, DSI& messages, unsigned char* img, float* lut);
    void scanlineOptimization_vert(DSI& dv, DSI& messages, unsigned char* img, float* lut);

    DSI m_dsi, messages;

    DSI messages_hor, messages_ver;

    float* wLUT;

    int m_w, m_h;

    float m_smoothness;
    float m_penalty1;
    float m_penalty2;
    float m_alpha;
    int m_minDisparity;
    int m_maxDisparity;
    int m_numDirections;
    int m_sgmConfidenceThreshold;
    int m_doSubPixRefinement;
    int m_doSequential;

public:
    SGMStereo(int _w, int _h, int minDisparity, int maxDisparity, int numDirections, int sgmConfidenceThreshold, int doSubPixRefinement,
              float smoothness,
              float penalty1,
              float penalty2,
              float alpha,
              int doSequential);

    void Run(unsigned char* iLeft, unsigned char* iRight, float* dispMap, unsigned char* confMap);

    void free();
};
#endif

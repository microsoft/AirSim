// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#ifndef dsimage_h
#define dsimage_h

#include <stdio.h>
#include <limits.h>
#include <float.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

class DSI
{
public:
    DSI()
    {
        m_cols = 0;
        m_rows = 0;
        m_planes = 0;
        m_data = NULL;
    }

    void create(uint64_t cols, uint64_t rows, uint64_t planes)
    {
        m_cols = cols;
        m_rows = rows;
        m_planes = planes;

        uint64_t pixelCount = m_cols * m_rows * m_planes;
        m_data = (short*)_aligned_malloc(pixelCount * sizeof(short), 16);
        if (!m_data) {
            printf("[ERROR] not enough memory!\n");
            exit(1);
        }
    }

    void setzero()
    {
        uint64_t pixelCount = m_cols * m_rows * m_planes;
        memset(m_data, 0, pixelCount * sizeof(short));
    }

    short operator()(uint64_t x, uint64_t y, uint64_t z) const
    {
        return m_data[(x + y * m_cols) * m_planes + z];
    }

    short& operator()(uint64_t x, uint64_t y, uint64_t z)
    {
        return m_data[(x + y * m_cols) * m_planes + z];
    }

    short* operator()(uint64_t x, uint64_t y) const
    {
        return &(m_data[(x + y * m_cols) * m_planes]);
    }

    void getDispMap(int confThreshold, int doSubPixRefinement, float* dispMap, unsigned char* confMap)
    {
        // first row
        {
            uint64_t offset = 0;
            float* pDisp = &(dispMap[offset]);
            unsigned char* pConf = &(confMap[offset]);
            for (int x = 0; x < m_cols; x++) {
                pDisp[x] = FLT_MAX;
                pConf[x] = 0;
            }
        }

        // last row
        {
            uint64_t offset = (m_rows - 1) * m_cols;
            float* pDisp = &(dispMap[offset]);
            unsigned char* pConf = &(confMap[offset]);
            for (int x = 0; x < m_cols; x++) {
                pDisp[x] = FLT_MAX;
                pConf[x] = 0;
            }
        }

        for (int y = 0; y < m_rows; y++) {
            uint64_t offset = y * m_cols;
            float* pDisp = &(dispMap[offset]);
            unsigned char* pConf = &(confMap[offset]);
            {
                pDisp[0] = FLT_MAX;
                pConf[0] = 0;
                pDisp[m_cols - 1] = FLT_MAX;
                pConf[m_cols - 1] = 0;
            }
        }

        for (int y = 1; y < m_rows - 1; y++) {
            uint64_t offset = y * m_cols;
            float* pDisp = &(dispMap[offset]);
            unsigned char* pConf = &(confMap[offset]);

#pragma omp parallel for schedule(dynamic, 1)

            for (int x = 1; x < m_cols - 1; x++) {
                int bestplane = (int)m_planes - 1;
                short minval = SHRT_MAX;
                short secondminval = SHRT_MAX;
                short* pV = (*this)(x, y);
                for (int d = 0; d < m_planes; d++) {
                    short val = pV[d];
                    if (val < minval) {
                        minval = val;
                        bestplane = d;
                    }
                }

                for (int d = 0; d < m_planes; d++) {
                    if (abs(d - bestplane) > 2) {
                        short val = pV[d];
                        if (val < secondminval) {
                            secondminval = val;
                        }
                    }
                }

                float distinctiveness1 = float(minval) / float(secondminval + 1e-9f);
                float conf = (float)__min(__max(20.0f * log(1.0f / (distinctiveness1 * distinctiveness1)), 0.0f), 255.0f);
                int Dim = (int)m_planes;
                if (conf >= confThreshold) {
                    // Local quadratic fit of cost and subpixel refinement.
                    double rDisp = bestplane;
                    double rCost = minval;
                    if (doSubPixRefinement) {
                        if (bestplane >= 1 && bestplane < m_planes - 1) {
                            double yl = pV[bestplane - 1];
                            double xc = bestplane;
                            double yc = minval;
                            double yu = pV[bestplane + 1];
                            double d2 = yu - yc + yl - yc;
                            double d1 = 0.5 * (yu - yl);
                            if (fabs(d2) > fabs(d1)) {
                                rDisp = xc - d1 / d2;
                                rCost = yc + 0.5 * d1 * (rDisp - xc);
                            }
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

    ~DSI()
    {
        free();
    }

    void free()
    {
        if (m_data != NULL)
            _aligned_free(m_data);
        m_data = NULL;
    }

    uint64_t m_cols;
    uint64_t m_rows;
    uint64_t m_planes;
    short* m_data;
};

void getDispMap2(DSI& dv1, DSI& dv2, int confThreshold, float* dispMap, unsigned char* confMap);

#endif

#if 0
void dump(vt::CByteImg& left, vt::CByteImg& right, int flag)
{
	int w = m_cols;
	int h = m_rows;
	for (int d = 0; d < m_planes; d++)
	{
		vt::CByteImg D;
		D.Create(w, h);
		D.Fill(byte(255));
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				{
					D(j, i) = (*this)(j, i, d);
				}
			}
		}
		vt::wstring fn;
		fn.format_with_resize(L"dsi%d/disp-%04d.png", flag, d);
		vt::VtSaveImage(fn, D);
	}
}
#endif
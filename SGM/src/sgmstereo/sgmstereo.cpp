// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// by Sudipta Sinha
// adapted for AirSim by Matthias Mueller

#include <stdint.h>
#include <vector>

#include "sgmstereo.h"
#include "dsimage.h"
#include "emmintrin.h"

SGMStereo::SGMStereo(int _w, int _h, int minDisparity, int maxDisparity, int numDirections, int sgmConfidenceThreshold, int doSubPixRefinement,
                     float smoothness,
                     float penalty1,
                     float penalty2,
                     float alpha,
                     int doSequential)
{
    m_w = _w;
    m_h = _h;
    m_smoothness = smoothness;
    m_penalty1 = penalty1;
    m_penalty2 = penalty2;
    m_alpha = alpha;
    m_minDisparity = minDisparity;
    m_maxDisparity = maxDisparity;
    m_numDirections = numDirections;
    m_sgmConfidenceThreshold = sgmConfidenceThreshold;
    m_doSubPixRefinement = doSubPixRefinement;
    m_doSequential = doSequential;

    if (minDisparity >= maxDisparity) {
        printf("[ERROR] Invalid Disparity Range [%d -- %d] ...\n", minDisparity, maxDisparity);
        exit(1);
    }

    int dispRange = maxDisparity - minDisparity;

    m_dsi.create(_w, _h, dispRange);

    if (m_doSequential) {
        messages.create(_w, _h, dispRange);
    }
    else {
        messages_hor.create(_w, _h, dispRange);
        messages_ver.create(_w, _h, dispRange);
    }

    float rec_penalty2 = 1.0f / m_penalty2;
    wLUT = new float[256];
    for (int i = 0; i < 256; i++) {
        wLUT[i] = (float)(m_penalty1 + m_alpha * exp(-i * rec_penalty2));
    }
}

void SGMStereo::calculateDSI_sse(unsigned char* L, unsigned char* R)
{
    int cols = m_w;
    int rows = m_h;

#pragma omp parallel for schedule(dynamic, 1)

    for (int y = 1; y < rows - 1; y++) {
        int off1 = cols * y;
        int off0 = off1 - cols;
        int off2 = off1 + cols;
        for (int x = 1; x < cols - 1; x++) {
            float u_00 = (float)L[off0 + x - 1];
            float u_10 = (float)L[off0 + x];
            float u_20 = (float)L[off0 + x + 1];
            float u_01 = (float)L[off1 + x - 1];
            float u_11 = (float)L[off1 + x];
            float u_21 = (float)L[off1 + x + 1];
            float u_02 = (float)L[off2 + x - 1];
            float u_12 = (float)L[off2 + x];
            float u_22 = (float)L[off2 + x + 1];

            for (int disp = m_minDisparity; disp < m_maxDisparity; disp += 4) {
                if (x + disp - 1 >= 0 && x + disp + 5 < m_w) {
                    float a[6], b[6], c[6];
                    for (int k = -1; k < 5; k++) {
                        a[k + 1] = (float)R[off0 + x + disp + k];
                        b[k + 1] = (float)R[off1 + x + disp + k];
                        c[k + 1] = (float)R[off2 + x + disp + k];
                    }

                    __m128 v_00 = _mm_set_ps(a[0], a[1], a[2], a[3]);
                    __m128 v_10 = _mm_set_ps(a[1], a[2], a[3], a[4]);
                    __m128 v_20 = _mm_set_ps(a[2], a[3], a[4], a[5]);
                    __m128 v_01 = _mm_set_ps(b[0], b[1], b[2], b[3]);
                    __m128 v_11 = _mm_set_ps(b[1], b[2], b[3], b[4]);
                    __m128 v_21 = _mm_set_ps(b[2], b[3], b[4], b[5]);
                    __m128 v_02 = _mm_set_ps(c[0], c[1], c[2], c[3]);
                    __m128 v_12 = _mm_set_ps(c[1], c[2], c[3], c[4]);
                    __m128 v_22 = _mm_set_ps(c[2], c[3], c[4], c[5]);

                    float sum_1 = (u_00 + u_10 + u_20 + u_01 + u_11 + u_21 + u_02 + u_12 + u_22) / 9.0f;
                    __m128 sum_2 = _mm_div_ps(_mm_add_ps(v_00, _mm_add_ps(v_10, _mm_add_ps(v_20, _mm_add_ps(v_01, _mm_add_ps(v_11, _mm_add_ps(v_21, _mm_add_ps(v_02, _mm_add_ps(v_12, v_22)))))))), _mm_set1_ps(9.0f));
                    __m128 val_1, val_2, sum_v1_x_v1, sum_v2_x_v2, sum_v1_x_v2;

                    val_1 = _mm_set1_ps(u_00 - sum_1);
                    val_2 = _mm_sub_ps(v_00, sum_2);
                    sum_v1_x_v1 = _mm_mul_ps(val_1, val_1);
                    sum_v2_x_v2 = _mm_mul_ps(val_2, val_2);
                    sum_v1_x_v2 = _mm_mul_ps(val_1, val_2);

                    val_1 = _mm_set1_ps(u_10 - sum_1);
                    val_2 = _mm_sub_ps(v_10, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_20 - sum_1);
                    val_2 = _mm_sub_ps(v_20, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_01 - sum_1);
                    val_2 = _mm_sub_ps(v_01, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_11 - sum_1);
                    val_2 = _mm_sub_ps(v_11, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_21 - sum_1);
                    val_2 = _mm_sub_ps(v_21, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_02 - sum_1);
                    val_2 = _mm_sub_ps(v_02, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_12 - sum_1);
                    val_2 = _mm_sub_ps(v_12, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    val_1 = _mm_set1_ps(u_22 - sum_1);
                    val_2 = _mm_sub_ps(v_22, sum_2);
                    sum_v1_x_v1 = _mm_add_ps(sum_v1_x_v1, _mm_mul_ps(val_1, val_1));
                    sum_v2_x_v2 = _mm_add_ps(sum_v2_x_v2, _mm_mul_ps(val_2, val_2));
                    sum_v1_x_v2 = _mm_add_ps(sum_v1_x_v2, _mm_mul_ps(val_1, val_2));

                    __m128 ncc1 = _mm_div_ps(sum_v1_x_v2, _mm_sqrt_ps(_mm_max_ps(_mm_mul_ps(sum_v1_x_v1, sum_v2_x_v2), _mm_set1_ps(0.01f))));
                    __m128 score = _mm_min_ps(_mm_mul_ps(_mm_sub_ps(_mm_set1_ps(1.0f), ncc1), _mm_set1_ps(255.0f)), _mm_set1_ps(255.0f));
                    float scores[4];
                    _mm_storeu_ps(scores, score);
                    short* pDSI = m_dsi(x, y);
                    int indexDisp = disp - m_minDisparity;
                    pDSI[indexDisp + 3] = (short)scores[0];
                    pDSI[indexDisp + 2] = (short)scores[1];
                    pDSI[indexDisp + 1] = (short)scores[2];
                    pDSI[indexDisp] = (short)scores[3];
                }
                else {
                    short* pDSI = m_dsi(x, y);
                    int indexDisp = disp - m_minDisparity;
                    pDSI[indexDisp + 3] = (short)255;
                    pDSI[indexDisp + 2] = (short)255;
                    pDSI[indexDisp + 1] = (short)255;
                    pDSI[indexDisp] = (short)255;
                }
            }
        }
    }
}

void subtractMinVal(short* pMessage, int size)
{
    __m128i* pM = (__m128i*)pMessage;
    __m128i minval = *pM;
    for (int i = 1; i < size / 8; i++) {
        pM++;
        minval = _mm_min_epi16(*pM, minval);
    }

    union u
    {
        __m128i m;
        short s[8];
    } x;

    x.m = minval;
    short val = __min(__min(__min(x.s[0], x.s[1]), __min(x.s[2], x.s[3])), __min(__min(x.s[4], x.s[5]), __min(x.s[6], x.s[7])));

    minval = _mm_set1_epi16(val);

    for (int i = 0; i < size / 8; i++) {
        *pM = _mm_sub_epi16(*pM, minval);
        pM--;
    }
}

void SGMStereo::messagePassing(short* pData, short* pBuffer1, short* pDMessage, int size, float weight, short smoothness)
{
    short pen1 = smoothness;
    short pen2 = (short)(smoothness * weight);

#if 1
    __m128i* pB1 = (__m128i*)pBuffer1;
    __m128i* pDM = (__m128i*)pDMessage;
    __m128i* pD = (__m128i*)pData;
    __m128i penalty1 = _mm_set1_epi16(pen1);
    __m128i penalty2 = _mm_set1_epi16(pen2);
    __m128i buffer1, buffer2, buffer3;

    subtractMinVal(pBuffer1, size);

    short newval(255 * 64), newval2(255 * 64);
    for (int i = 0; i < size / 8; i++) {
        buffer1 = _mm_slli_si128(*pB1, 2);
        buffer3 = _mm_insert_epi16(buffer1, newval, 0);
        buffer1 = _mm_add_epi16(buffer3, penalty1);

        buffer2 = _mm_srli_si128(*pB1, 2);
        if (i * 8 + 8 < size) {
            newval = pBuffer1[i * 8 + 7];
            newval2 = pBuffer1[i * 8 + 8];
        }
        else
            newval2 = 255 * 64;

        buffer3 = _mm_insert_epi16(buffer2, newval2, 7);
        buffer2 = _mm_add_epi16(buffer3, penalty1);
        buffer3 = _mm_min_epi16(buffer1, _mm_min_epi16(buffer2, _mm_min_epi16(*pB1, penalty2)));

        *pB1 = _mm_add_epi16(buffer3, *pD);
        *pDM = _mm_add_epi16(*pDM, *pB1);

        pB1++;
        pDM++;
        pD++;
    }
#else
    int limit = 32768;
    short mem[2048];
    short minval = limit - 1;

    for (int i = 0; i < size; i++) {
        minval = __min(minval, pBuffer1[i]);
    }

    for (int i = 0; i < size; i++) {
        pBuffer1[i] -= minval;
    }

    for (int i = 0; i < size; i++) {
        short minval1 = __min(pBuffer1[__max(i - 1, 0)], pBuffer1[__min(i + 1, size - 1)]) + pen1;
        short minval2 = __min(pBuffer1[i], pen2);
        mem[i] = __min(minval1, minval2);
    }

    for (int i = 0; i < size; i++) {
        short val = mem[i] + pData[i];
        if (val + pDMessage[i] >= limit) {
            pDMessage[i] = limit - 1;
        }
        else {
            pDMessage[i] += val;
        }
        pBuffer1[i] = val;
    }
#endif
}

void SGMStereo::scanlineOptimization(DSI& dv, DSI& msgs, unsigned char* img, float* lut, int dx_, int dy_)
{
    int cols = (int)dv.m_cols;
    int rows = (int)dv.m_rows;
    int planes = (int)dv.m_planes;

    std::vector<float> startx;
    std::vector<float> starty;
    startx.reserve(2 * rows + 2 * cols);
    starty.reserve(2 * rows + 2 * cols);

    for (int ys = 0; ys < rows; ys++) {
        for (int xs = 0; xs < cols; xs++) {
            if ((xs == 0 && dx_ == 1) || (ys == 0 && dy_ == 1) || (ys == rows - 1 && dy_ == -1) || (xs == cols - 1 && dx_ == -1)) {
                startx.push_back(float(xs));
                starty.push_back(float(ys));
            }
        }
    }

    short* buffervec = (short*)_aligned_malloc(planes * sizeof(short), 16);

    float dist = (float)sqrt((dx_ * dx_ + dy_ * dy_));

    for (int j = 0; j < (int)startx.size(); j++) {
        int x = (int)startx[j];
        int y = (int)starty[j];
        //uint64_t x64 = (uint64_t)x;
        //uint64_t y64 = (uint64_t)y;

        int dx = dx_;
        int dy = dy_;

        for (int i = 0; i < planes; i++) {
            buffervec[i] = 0;
        }

        short smoothness = (short)(m_smoothness / dist);
        bool forward = true;

        int oldColor = 0;
        do {
            int newIntensity = img[y * cols + x];
            int diff = abs(newIntensity - oldColor);
            oldColor = newIntensity;
            float weight = lut[diff];

            messagePassing(dv(x, y), buffervec, msgs(x, y), planes, weight, smoothness);

            y += dy;
            x += dx;

            if (forward && (y == rows || x == cols || y < 0)) {
                dx *= -1;
                dy *= -1;
                y += dy;
                x += dx;
                for (int i = 0; i < planes; i++)
                    buffervec[i] = 0;
                forward = false;
            }
        } while (forward || (y >= 0 && x >= 0 && y < rows && x < cols));

        dx *= -1;
        dy *= -1;
    }

    _aligned_free(buffervec);
}

void SGMStereo::scanlineOptimization_hor(DSI& dv, DSI& msgs, unsigned char* img, float* lut)
{
    int cols = (int)dv.m_cols;
    int rows = (int)dv.m_rows;
    int planes = (int)dv.m_planes;
    int bufsize = planes * sizeof(short);
    short* buf = (short*)_aligned_malloc(bufsize, 16);
    short smoothness = (short)(m_smoothness);

    for (int y = 0; y < rows; y++) {
        int offset = y * cols;
        int oldIntensity = 0;
        memset(buf, 0, bufsize);
        for (int x = 0; x < cols; x++) {
            int newIntensity = img[offset + x];
            int diff = abs(newIntensity - oldIntensity);
            oldIntensity = newIntensity;
            float weight = lut[diff];
            messagePassing(dv(x, y), buf, msgs(x, y), int(planes), weight, smoothness);
        }
        oldIntensity = 0;
        memset(buf, 0, bufsize);
        for (int x = cols - 1; x >= 0; x--) {
            int newIntensity = img[offset + x];
            int diff = abs(newIntensity - oldIntensity);
            oldIntensity = newIntensity;
            float weight = lut[diff];
            messagePassing(dv(x, y), buf, msgs(x, y), planes, weight, smoothness);
        }
    }
    _aligned_free(buf);
}

void SGMStereo::scanlineOptimization_vert(DSI& dv, DSI& msgs, unsigned char* img, float* lut)
{
    int cols = (int)dv.m_cols;
    int rows = (int)dv.m_rows;
    int planes = (int)dv.m_planes;
    int bufsize = planes * sizeof(short);
    short* buf = (short*)_aligned_malloc(bufsize, 16);
    short smoothness = (short)(m_smoothness);
    for (int x = 0; x < cols; x++) {
        int offset = x;
        int oldIntensity = 0;
        memset(buf, 0, bufsize);
        for (int y = 0; y < rows; y++) {
            int newIntensity = img[offset];
            int diff = abs(newIntensity - oldIntensity);
            oldIntensity = newIntensity;
            float weight = lut[diff];
            messagePassing(dv(x, y), buf, msgs(x, y), planes, weight, smoothness);
            offset += cols;
        }

        offset = cols * (rows - 1) + x;
        oldIntensity = 0;
        memset(buf, 0, bufsize);
        for (int y = rows - 1; y > 0; y--) {
            int newIntensity = img[offset];
            int diff = abs(newIntensity - oldIntensity);
            oldIntensity = newIntensity;
            float weight = lut[diff];
            messagePassing(dv(x, y), buf, msgs(x, y), planes, weight, smoothness);
            offset -= cols;
        }
    }
    _aligned_free(buf);
}

void SGMStereo::Run(
    unsigned char* iLeft,
    unsigned char* iRight,
    float* dispMap,
    unsigned char* confMap)
{
    calculateDSI_sse(iLeft, iRight);

    if (m_doSequential) {
        messages.setzero();
        scanlineOptimization_hor(m_dsi, messages, iLeft, wLUT);
        scanlineOptimization_vert(m_dsi, messages, iLeft, wLUT);
        if (m_numDirections == 8) {
            scanlineOptimization(m_dsi, messages, iLeft, wLUT, 1, 1);
            scanlineOptimization(m_dsi, messages, iLeft, wLUT, 1, -1);
        }
        messages.getDispMap(m_sgmConfidenceThreshold, m_doSubPixRefinement, dispMap, confMap);
    }
    else {
        messages_hor.setzero();
        messages_ver.setzero();

#pragma omp parallel for
        for (int k = 0; k < 2; k++) {
            if (k == 0)
                scanlineOptimization_hor(m_dsi, messages_hor, iLeft, wLUT);
            if (k == 1)
                scanlineOptimization_vert(m_dsi, messages_ver, iLeft, wLUT);
        }
        getDispMap2(messages_hor, messages_ver, m_sgmConfidenceThreshold, dispMap, confMap);
    }
}

void SGMStereo::free()
{
    m_dsi.free();

    if (m_doSequential) {
        messages.free();
    }
    else {
        messages_hor.free();
        messages_ver.free();
    }

    delete[] wLUT;
}

#if 0
void SGMStereo::calculateDSI(CByteImg& refImage, CByteImg& nbrImage)
{
	int cols = refImage.Width();
	int rows = refImage.Height();

	const int nw = nbrImage.Width();
	const int nh = nbrImage.Height();

	int disparityRange = m_maxDisparity - m_minDisparity;

	printf("\n   Calculate DSI (%4d x %4d x %4d), disparity-range=(%d %d)\n\n", cols, rows, disparityRange, m_minDisparity, m_maxDisparity);

	m_dsi.create(cols, rows, disparityRange, 255);

	for (int disp = m_minDisparity; disp < m_maxDisparity; disp++)
	{
#pragma omp parallel for schedule(dynamic, 1)
		for (int y = 1; y < rows - 1; y++)
		{
			float L10_ = refImage(-1 + 1, -1 + y);
			float L20_ = refImage(0 + 1, -1 + y);
			float L11_ = refImage(-1 + 1, 0 + y);
			float L21_ = refImage(0 + 1, 0 + y);
			float L12_ = refImage(-1 + 1, +1 + y);
			float L22_ = refImage(0 + 1, +1 + y);

			float R10_ = 0;
			float R20_ = 0;
			float R11_ = 0;
			float R21_ = 0;
			float R12_ = 0;
			float R22_ = 0;
			if (disp >= 0 && disp < nw - 1)
			{
				R10_ = nbrImage(-1 + disp + 1, -1 + y);
				R20_ = nbrImage(0 + disp + 1, -1 + y);
				R11_ = nbrImage(-1 + disp + 1, 0 + y);
				R21_ = nbrImage(0 + disp + 1, 0 + y);
				R12_ = nbrImage(-1 + disp + 1, +1 + y);
				R22_ = nbrImage(0 + disp + 1, +1 + y);
			}
			for (int x = 1; x < cols - 1; x++)
			{
				float L00_ = L10_;
				float L01_ = L11_;
				float L02_ = L12_;

				L10_ = L20_;
				L11_ = L21_;
				L12_ = L22_;

				L20_ = refImage(+1 + x, -1 + y);
				L21_ = refImage(+1 + x, 0 + y);
				L22_ = refImage(+1 + x, +1 + y);

				float Lavg = (L00_ + L10_ + L20_ + L01_ + L11_ + L21_ + L02_ + L12_ + L22_) / 9.0f;

				float L00 = L00_ - Lavg; float L10 = L10_ - Lavg; float L20 = L20_ - Lavg;
				float L01 = L01_ - Lavg; float L11 = L11_ - Lavg; float L21 = L21_ - Lavg;
				float L02 = L02_ - Lavg; float L12 = L12_ - Lavg; float L22 = L22_ - Lavg;

				float LL = L00 * L00 + L10 * L10 + L20 * L20 + L01 * L01 + L11 * L11 + L21 * L21 + L02 * L02 + L12 * L12 + L22 * L22;

				float R00_ = R10_;
				float R01_ = R11_;
				float R02_ = R12_;

				R10_ = R20_;
				R11_ = R21_;
				R12_ = R22_;

				if (1 + x + disp < 0 || +1 + x + disp >= nw)
				{
					int indexDisp = disp - m_minDisparity;
					m_dsi(x, y, indexDisp) = short(255);
					continue;
				}
				R20_ = nbrImage(+1 + x + disp, -1 + y);
				R21_ = nbrImage(+1 + x + disp, 0 + y);
				R22_ = nbrImage(+1 + x + disp, +1 + y);

				float Ravg = (R00_ + R10_ + R20_ + R01_ + R11_ + R21_ + R02_ + R12_ + R22_) / 9.0f;

				float R00 = R00_ - Ravg; float R10 = R10_ - Ravg; float R20 = R20_ - Ravg;
				float R01 = R01_ - Ravg; float R11 = R11_ - Ravg; float R21 = R21_ - Ravg;
				float R02 = R02_ - Ravg; float R12 = R12_ - Ravg; float R22 = R22_ - Ravg;

				float RR = R00 * R00 + R10 * R10 + R20 * R20 + R01 * R01 + R11 * R11 + R21 * R21 + R02 * R02 + R12 * R12 + R22 * R22;

				float LR = L00 * R00 + L10 * R10 + L20 * R20 + L01 * R01 + L11 * R11 + L21 * R21 + L02 * R02 + L12 * R12 + L22 * R22;

				float ncc = LR / VtMax(sqrt(LL * RR), 1e-2f);
				int indexDisp = disp - m_minDisparity;
				int cost = (int)VtMin((1.0f - ncc) * 255.0f, 255.0f);
				m_dsi(x, y, indexDisp) = cost;

			} //for x
		} //for y
	} //for disp
}
#endif

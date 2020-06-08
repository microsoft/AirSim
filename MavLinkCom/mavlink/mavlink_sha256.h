#pragma once

/*
  sha-256 implementation for MAVLink based on Heimdal sources, with
  modifications to suit mavlink headers
 */
/*
 * Copyright (c) 1995 - 2001 Kungliga Tekniska Högskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
  allow implementation to provide their own sha256 with the same API
*/
#ifndef HAVE_MAVLINK_SHA256

#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

typedef struct {
  uint32_t sz[2];
  uint32_t counter[8];
  union {
      unsigned char save_bytes[64];
      uint32_t save_u32[16];
  } u;
} mavlink_sha256_ctx;

#define Ch(x,y,z) (((x) & (y)) ^ ((~(x)) & (z)))
#define Maj(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define ROTR(x,n)   (((x)>>(n)) | ((x) << (32 - (n))))

#define Sigma0(x)	(ROTR(x,2)  ^ ROTR(x,13) ^ ROTR(x,22))
#define Sigma1(x)	(ROTR(x,6)  ^ ROTR(x,11) ^ ROTR(x,25))
#define sigma0(x)	(ROTR(x,7)  ^ ROTR(x,18) ^ ((x)>>3))
#define sigma1(x)	(ROTR(x,17) ^ ROTR(x,19) ^ ((x)>>10))

#define A m->counter[0]
#define B m->counter[1]
#define C m->counter[2]
#define D m->counter[3]
#define E m->counter[4]
#define F m->counter[5]
#define G m->counter[6]
#define H m->counter[7]

static const uint32_t mavlink_sha256_constant_256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

MAVLINK_HELPER void mavlink_sha256_init(mavlink_sha256_ctx *m)
{
    m->sz[0] = 0;
    m->sz[1] = 0;
    A = 0x6a09e667;
    B = 0xbb67ae85;
    C = 0x3c6ef372;
    D = 0xa54ff53a;
    E = 0x510e527f;
    F = 0x9b05688c;
    G = 0x1f83d9ab;
    H = 0x5be0cd19;
}

static inline void mavlink_sha256_calc(mavlink_sha256_ctx *m, uint32_t *in)
{
    uint32_t AA, BB, CC, DD, EE, FF, GG, HH;
    uint32_t data[64];
    int i;

    AA = A;
    BB = B;
    CC = C;
    DD = D;
    EE = E;
    FF = F;
    GG = G;
    HH = H;

    for (i = 0; i < 16; ++i)
	data[i] = in[i];
    for (i = 16; i < 64; ++i)
	data[i] = sigma1(data[i-2]) + data[i-7] + 
	    sigma0(data[i-15]) + data[i - 16];

    for (i = 0; i < 64; i++) {
	uint32_t T1, T2;

	T1 = HH + Sigma1(EE) + Ch(EE, FF, GG) + mavlink_sha256_constant_256[i] + data[i];
	T2 = Sigma0(AA) + Maj(AA,BB,CC);
			     
	HH = GG;
	GG = FF;
	FF = EE;
	EE = DD + T1;
	DD = CC;
	CC = BB;
	BB = AA;
	AA = T1 + T2;
    }

    A += AA;
    B += BB;
    C += CC;
    D += DD;
    E += EE;
    F += FF;
    G += GG;
    H += HH;
}

MAVLINK_HELPER void mavlink_sha256_update(mavlink_sha256_ctx *m, const void *v, uint32_t len)
{
    const unsigned char *p = (const unsigned char *)v;
    uint32_t old_sz = m->sz[0];
    uint32_t offset;

    m->sz[0] += len * 8;
    if (m->sz[0] < old_sz)
	++m->sz[1];
    offset = (old_sz / 8) % 64;
    while(len > 0){
	uint32_t l = 64 - offset;
        if (len < l) {
            l = len;
        }
	memcpy(m->u.save_bytes + offset, p, l);
	offset += l;
	p += l;
	len -= l;
	if(offset == 64){
	    int i;
	    uint32_t current[16];
	    const uint32_t *u = m->u.save_u32;
	    for (i = 0; i < 16; i++){
                const uint8_t *p1 = (const uint8_t *)&u[i];
                uint8_t *p2 = (uint8_t *)&current[i];
                p2[0] = p1[3];
                p2[1] = p1[2];
                p2[2] = p1[1];
                p2[3] = p1[0];
	    }
	    mavlink_sha256_calc(m, current);
	    offset = 0;
	}
    }
}

/*
  get first 48 bits of final sha256 hash
 */
MAVLINK_HELPER void mavlink_sha256_final_48(mavlink_sha256_ctx *m, uint8_t result[6])
{
    unsigned char zeros[72];
    unsigned offset = (m->sz[0] / 8) % 64;
    unsigned int dstart = (120 - offset - 1) % 64 + 1;
    uint8_t *p = (uint8_t *)&m->counter[0];
    
    *zeros = 0x80;
    memset (zeros + 1, 0, sizeof(zeros) - 1);
    zeros[dstart+7] = (m->sz[0] >> 0) & 0xff;
    zeros[dstart+6] = (m->sz[0] >> 8) & 0xff;
    zeros[dstart+5] = (m->sz[0] >> 16) & 0xff;
    zeros[dstart+4] = (m->sz[0] >> 24) & 0xff;
    zeros[dstart+3] = (m->sz[1] >> 0) & 0xff;
    zeros[dstart+2] = (m->sz[1] >> 8) & 0xff;
    zeros[dstart+1] = (m->sz[1] >> 16) & 0xff;
    zeros[dstart+0] = (m->sz[1] >> 24) & 0xff;

    mavlink_sha256_update(m, zeros, dstart + 8);

    // this ordering makes the result consistent with taking the first
    // 6 bytes of more conventional sha256 functions. It assumes
    // little-endian ordering of m->counter
    result[0] = p[3];
    result[1] = p[2];
    result[2] = p[1];
    result[3] = p[0];
    result[4] = p[7];
    result[5] = p[6];
}

// prevent conflicts with users of the header
#undef A
#undef B
#undef C
#undef D
#undef E
#undef F
#undef G
#undef H
#undef Ch
#undef ROTR
#undef Sigma0
#undef Sigma1
#undef sigma0
#undef sigma1

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif

#endif // HAVE_MAVLINK_SHA256

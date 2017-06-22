#pragma once

#include <cstdlib>
#include <cstdint>
#include <cmath>

#ifdef __GNUC__
_Pragma("GCC diagnostic push")
_Pragma("GCC diagnostic ignored \"-Wstrict-aliasing\"")    
#endif

#if defined(_MSC_VER)
__pragma(warning(push))	
__pragma(warning( disable : 4100 4028 4189 4204 4244 4245 4239 4464 4456 4505 4514 4571 4624 4626 4267 4710 4820 5027 5031))					  
#endif

typedef struct
{
  float x;
  float y;
  float z;
} vector_t;

typedef struct
{
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} intvec_t;

typedef struct
{
  int32_t w;
  int32_t x;
  int32_t y;
  int32_t z;
} intquat_t;

int32_t int_dot(intvec_t v1, intvec_t v2);
intvec_t int_cross(intvec_t u, intvec_t v);
intvec_t int_scalar_multiply(int32_t s, intvec_t v);
intvec_t int_vector_add(intvec_t u, intvec_t v);
intvec_t int_vector_sub(intvec_t u, intvec_t v);
int32_t int_sqrd_norm(intvec_t v);
intvec_t int_vector_normalize(intvec_t v);
intquat_t int_quaternion_normalize(intquat_t q);
intquat_t int_quaternion_multiply(const intquat_t q1, const intquat_t q2);
intquat_t int_quaternion_inverse(intquat_t q);
intquat_t int_quaternion_from_two_vectors(intvec_t u, intvec_t v);

float dot(vector_t v1, vector_t v2);
vector_t cross(vector_t u, vector_t v);
vector_t scalar_multiply(float s, vector_t v);
vector_t vector_add(vector_t u, vector_t v);
vector_t vector_sub(vector_t u, vector_t v);
float sqrd_norm(vector_t v);
vector_t vector_normalize(vector_t v);
quaternion_t quaternion_normalize(quaternion_t q);
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_inverse(quaternion_t q);
quaternion_t quat_from_two_vectors(vector_t u, vector_t v);

void euler_from_quat(quaternion_t q, float *phi, float *theta, float *psi);
void euler_from_int_quat(intquat_t q, int32_t phi, int32_t theta, int32_t psi);

float turboInvSqrt(float x);

float fsat(float value, float max);
int32_t sat(int32_t value, int32_t max);
float fsign(float y);


/************************************************** Implementation ***************************************************************/


//void pfquat() __attribute__ ((unused));
void pfquat(){}

int32_t int_dot(intvec_t v1, intvec_t v2)
{
    return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z)/1000;
}

intvec_t int_cross(intvec_t u, intvec_t v)
{
    intvec_t out;
    out.x = (u.y*v.z - u.z*v.y)/1000;
    out.y = (u.z*v.x - u.x*v.z)/1000;
    out.z = (u.x*v.y - u.y*v.x)/1000;
    return out;
}

intvec_t int_scalar_multiply(int32_t s, intvec_t v)
{
    intvec_t out;
    out.x = (s*v.x)/1000;
    out.y = (s*v.y)/1000;
    out.z = (s*v.z)/1000;
    return out;
}

intvec_t int_vector_add(intvec_t u, intvec_t v)
{
    intvec_t out;
    out.x = u.x + v.x;
    out.y = u.y + v.y;
    out.z = u.z + v.z;
    return out;
}

intvec_t int_vector_sub(intvec_t u, intvec_t v)
{
    intvec_t out;
    out.x = u.x - v.x;
    out.y = u.y - v.y;
    out.z = u.z - v.z;
    return out;
}

int32_t int_sqrd_norm(intvec_t v)
{
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

intvec_t int_vector_normalize(intvec_t v)
{
    float recipNorm = turboInvSqrt((v.x*v.x + v.y*v.y + v.z*v.z)/1000000.0f);
    return int_scalar_multiply((int32_t)(1000*recipNorm), v);
}

intquat_t int_quaternion_normalize(intquat_t q)
{
    float recipNorm = turboInvSqrt((q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) / 1000000.0f);
    int32_t intRecipNorm = (int32_t)(recipNorm*1000);
    intquat_t out;
    out.w = (intRecipNorm*q.w)/1000;
    out.x = (intRecipNorm*q.x)/1000;
    out.y = (intRecipNorm*q.y)/1000;
    out.z = (intRecipNorm*q.z)/1000;
    return out;
}

intquat_t int_quaternion_multiply(intquat_t q1, intquat_t q2)
{
    int32_t s1 = q1.w;
    int32_t s2 = q2.w;

    intvec_t v1 = {q1.x, q1.y, q2.z};
    intvec_t v2 = {q2.x, q2.y, q2.z};

    int32_t w = (s1*s2)/1000 - int_dot(v1, v2);
    // xyz = s1*v2 + s2*v1 - v1 x v2)
    intvec_t xyz = int_vector_sub(int_vector_add(int_scalar_multiply(s1, v2), int_scalar_multiply(s2,v1)), int_cross(v1,
        v2));
    intquat_t q = {w, xyz.x, xyz.y, xyz.z};
    return int_quaternion_normalize(q);
}


intquat_t int_quaternion_inverse(intquat_t q)
{

    intquat_t out = {q.w, -1*q.x, -1*q.y, -1*q.z};
    return out;
}

intquat_t int_quaternion_from_two_vectors(intvec_t u, intvec_t v)
{
    u = int_vector_normalize(u);
    v = int_vector_normalize(v);

    intvec_t half = int_vector_normalize(int_vector_add(u,v));

    int32_t w = int_dot(u, half);
    intvec_t xyz = int_cross(u,v);
    intquat_t q = {w, xyz.x, xyz.y, xyz.z};
    return int_quaternion_normalize(q);
}


/**************************************/
/*** FLOATING POINT IMPLEMENTATIONS ***/
/**************************************/
float dot(vector_t v1, vector_t v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

vector_t cross(vector_t u, vector_t v)
{
    vector_t out = {u.y *v.z - u.z*v.y,
        u.z *v.x - u.x*v.z,
        u.x *v.y - u.y *v.x
    };
    return out;
}

vector_t scalar_multiply(float s, vector_t v)
{
    vector_t out = {s*v.x,
        s*v.y,
        s *v.z
    };
    return out;
}

vector_t vector_add(vector_t u, vector_t v)
{
    vector_t out = {u.x + v.x,
        u.y + v.y,
        u.z + v.z
    };
    return out;

}

vector_t vector_sub(vector_t u, vector_t v)
{
    vector_t out = {u.x - v.x,
        u.y - v.y,
        u.z - v.z
    };
    return out;
}

float sqrd_norm(vector_t v)
{
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

float norm(vector_t v)
{
    float out = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    return out;
}

vector_t vector_normalize(vector_t v)
{
    float recipNorm = turboInvSqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    vector_t out = {recipNorm*v.x,
        recipNorm*v.y,
        recipNorm *v.z
    };
    return out;
}

quaternion_t quaternion_normalize(quaternion_t q)
{
    float recipNorm = turboInvSqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    quaternion_t out = {recipNorm*q.w,
        recipNorm*q.x,
        recipNorm*q.y,
        recipNorm *q.z
    };
    return out;
}

quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2)
{
    quaternion_t q = {q1.w *q2.w - q1.x *q2.x - q1.y *q2.y - q1.z*q2.z,
        q1.w *q2.x + q1.x *q2.w - q1.y *q2.z + q1.z*q2.y,
        q1.w *q2.y + q1.x *q2.z + q1.y *q2.w - q1.z*q2.x,
        q1.w *q2.z - q1.x *q2.y + q1.y *q2.x + q1.z *q2.w
    };
    return q;
}

quaternion_t quaternion_inverse(quaternion_t q)
{
    q.x *= -1.0f;
    q.y *= -1.0f;
    q.z *= -1.0f;
    return q;
}

quaternion_t quat_from_two_vectors(vector_t u, vector_t v)
{
    float w = 1.0f + dot(u, v);
    vector_t xyz = cross(u, v);
    quaternion_t q = {w, xyz.x, xyz.y, xyz.z};
    return quaternion_normalize(q);
}

void euler_from_quat(quaternion_t q, float *phi, float *theta, float *psi)
{
    *phi = atan2_approx(2.0f * (q.w*q.x + q.y*q.z),
        1.0f - 2.0f * (q.x*q.x + q.y*q.y));
    *theta = asin_approx(2.0f*(q.w*q.y - q.z*q.x));
    *psi = atan2_approx(2.0f * (q.w*q.z + q.x*q.y),
        1.0f - 2.0f * (q.y*q.y + q.z*q.z));
}


float turboInvSqrt(float x)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = x * 0.5F;
    y  = x;
    i  = * (long *) &y;                         // evil floating point bit level hacking
    i  = 0x5f3759df - (i >> 1);
    y  = * (float *) &i;
    y  = y * (threehalfs - (x2 * y * y));       // 1st iteration
    y  = y * (threehalfs - (x2 * y * y));       // 2nd iteration, this can be removed

    return y;
}

float fsign(float y)
{
    return (float)((0 < y) - (y < 0));
}


float fsat(float value, float max)
{
    if (fabs(value) > fabs(max))
    {
        value = max*fsign(value);
    }
    return value;
}


int32_t sat(int32_t value, int32_t max)
{
    if (abs(value) > abs(max))
    {
        value = max*sign(value);
    }
    return value;
}


#ifdef __GNUC__
_Pragma("GCC diagnostic pop")          
#endif

#if defined(_MSC_VER)
__pragma(warning(pop))					  
#endif

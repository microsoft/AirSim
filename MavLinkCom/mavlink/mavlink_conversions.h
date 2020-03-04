#pragma once

#ifndef MAVLINK_NO_CONVERSION_HELPERS

/* enable math defines on Windows */
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>

#ifndef M_PI_2
    #define M_PI_2 ((float)asin(1))
#endif

/**
 * @file mavlink_conversions.h
 *
 * These conversion functions follow the NASA rotation standards definition file
 * available online.
 *
 * Their intent is to lower the barrier for MAVLink adopters to use gimbal-lock free
 * (both rotation matrices, sometimes called DCM, and quaternions are gimbal-lock free)
 * rotation representations. Euler angles (roll, pitch, yaw) will be phased out of the
 * protocol as widely as possible.
 *
 * @author James Goppert
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */


/**
 * Converts a quaternion to a rotation matrix
 *
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 * @param dcm a 3x3 rotation matrix
 */
MAVLINK_HELPER void mavlink_quaternion_to_dcm(const float quaternion[4], float dcm[3][3])
{
    double a = quaternion[0];
    double b = quaternion[1];
    double c = quaternion[2];
    double d = quaternion[3];
    double aSq = a * a;
    double bSq = b * b;
    double cSq = c * c;
    double dSq = d * d;
    dcm[0][0] = aSq + bSq - cSq - dSq;
    dcm[0][1] = 2 * (b * c - a * d);
    dcm[0][2] = 2 * (a * c + b * d);
    dcm[1][0] = 2 * (b * c + a * d);
    dcm[1][1] = aSq - bSq + cSq - dSq;
    dcm[1][2] = 2 * (c * d - a * b);
    dcm[2][0] = 2 * (b * d - a * c);
    dcm[2][1] = 2 * (a * b + c * d);
    dcm[2][2] = aSq - bSq - cSq + dSq;
}


/**
 * Converts a rotation matrix to euler angles
 *
 * @param dcm a 3x3 rotation matrix
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 */
MAVLINK_HELPER void mavlink_dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw)
{
    float phi, theta, psi;
    theta = asinf(-dcm[2][0]);

    if (fabsf(theta - (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = (atan2f(dcm[1][2] - dcm[0][1],
                dcm[0][2] + dcm[1][1]) + phi);

    } else if (fabsf(theta + (float)M_PI_2) < 1.0e-3f) {
        phi = 0.0f;
        psi = atan2f(dcm[1][2] - dcm[0][1],
                  dcm[0][2] + dcm[1][1] - phi);

    } else {
        phi = atan2f(dcm[2][1], dcm[2][2]);
        psi = atan2f(dcm[1][0], dcm[0][0]);
    }

    *roll = phi;
    *pitch = theta;
    *yaw = psi;
}


/**
 * Converts a quaternion to euler angles
 *
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 */
MAVLINK_HELPER void mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw)
{
    float dcm[3][3];
    mavlink_quaternion_to_dcm(quaternion, dcm);
    mavlink_dcm_to_euler((const float(*)[3])dcm, roll, pitch, yaw);
}


/**
 * Converts euler angles to a quaternion
 *
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 */
MAVLINK_HELPER void mavlink_euler_to_quaternion(float roll, float pitch, float yaw, float quaternion[4])
{
    float cosPhi_2 = cosf(roll / 2);
    float sinPhi_2 = sinf(roll / 2);
    float cosTheta_2 = cosf(pitch / 2);
    float sinTheta_2 = sinf(pitch / 2);
    float cosPsi_2 = cosf(yaw / 2);
    float sinPsi_2 = sinf(yaw / 2);
    quaternion[0] = (cosPhi_2 * cosTheta_2 * cosPsi_2 +
            sinPhi_2 * sinTheta_2 * sinPsi_2);
    quaternion[1] = (sinPhi_2 * cosTheta_2 * cosPsi_2 -
            cosPhi_2 * sinTheta_2 * sinPsi_2);
    quaternion[2] = (cosPhi_2 * sinTheta_2 * cosPsi_2 +
            sinPhi_2 * cosTheta_2 * sinPsi_2);
    quaternion[3] = (cosPhi_2 * cosTheta_2 * sinPsi_2 -
            sinPhi_2 * sinTheta_2 * cosPsi_2);
}


/**
 * Converts a rotation matrix to a quaternion
 * Reference:
 *  - Shoemake, Quaternions,
 *  http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
 *
 * @param dcm a 3x3 rotation matrix
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 */
MAVLINK_HELPER void mavlink_dcm_to_quaternion(const float dcm[3][3], float quaternion[4])
{
    float tr = dcm[0][0] + dcm[1][1] + dcm[2][2];
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f);
        quaternion[0] = s * 0.5f;
        s = 0.5f / s;
        quaternion[1] = (dcm[2][1] - dcm[1][2]) * s;
        quaternion[2] = (dcm[0][2] - dcm[2][0]) * s;
        quaternion[3] = (dcm[1][0] - dcm[0][1]) * s;
    } else {
        /* Find maximum diagonal element in dcm
         * store index in dcm_i */
        int dcm_i = 0;
        int i;
        for (i = 1; i < 3; i++) {
            if (dcm[i][i] > dcm[dcm_i][dcm_i]) {
                dcm_i = i;
            }
        }

        int dcm_j = (dcm_i + 1) % 3;
        int dcm_k = (dcm_i + 2) % 3;

        float s = sqrtf((dcm[dcm_i][dcm_i] - dcm[dcm_j][dcm_j] -
                    dcm[dcm_k][dcm_k]) + 1.0f);
        quaternion[dcm_i + 1] = s * 0.5f;
        s = 0.5f / s;
        quaternion[dcm_j + 1] = (dcm[dcm_i][dcm_j] + dcm[dcm_j][dcm_i]) * s;
        quaternion[dcm_k + 1] = (dcm[dcm_k][dcm_i] + dcm[dcm_i][dcm_k]) * s;
        quaternion[0] = (dcm[dcm_k][dcm_j] - dcm[dcm_j][dcm_k]) * s;
    }
}


/**
 * Converts euler angles to a rotation matrix
 *
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 * @param dcm a 3x3 rotation matrix
 */
MAVLINK_HELPER void mavlink_euler_to_dcm(float roll, float pitch, float yaw, float dcm[3][3])
{
    float cosPhi = cosf(roll);
    float sinPhi = sinf(roll);
    float cosThe = cosf(pitch);
    float sinThe = sinf(pitch);
    float cosPsi = cosf(yaw);
    float sinPsi = sinf(yaw);

    dcm[0][0] = cosThe * cosPsi;
    dcm[0][1] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
    dcm[0][2] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

    dcm[1][0] = cosThe * sinPsi;
    dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
    dcm[1][2] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

    dcm[2][0] = -sinThe;
    dcm[2][1] = sinPhi * cosThe;
    dcm[2][2] = cosPhi * cosThe;
}

#endif // MAVLINK_NO_CONVERSION_HELPERS

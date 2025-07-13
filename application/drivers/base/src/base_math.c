/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    base_math.c
  * @author  MCU Driver Team
  * @brief   Provides functions about math.
  */

/* Includes ------------------------------------------------------------------ */
#include "base_math.h"

/* Private macro ------------------------------------------------------------- */
#if (BASE_MATH_SINCOS_MIDDLE_TABLE == BASE_CFG_ENABLE)
#define TRIGONOMETRIC_MAPPING_TABLE { \
    0X0000, 0X012D, 0X01F6, 0X02BF, 0X0388, 0X0451, 0X051A, 0X05E3, \
    0X06AC, 0X0775, 0X083D, 0X0906, 0X09CE, 0X0A97, 0X0B5F, 0X0C27, \
    0X0CEF, 0X0DB7, 0X0E7F, 0X0F47, 0X100E, 0X10D6, 0X119D, 0X1264, \
    0X132B, 0X13F2, 0X14B8, 0X157F, 0X1645, 0X170A, 0X17D0, 0X1896, \
    0X195B, 0X1A20, 0X1AE4, 0X1BA9, 0X1C6D, 0X1D31, 0X1DF5, 0X1EB8, \
    0X1F7B, 0X203E, 0X2100, 0X21C2, 0X2284, 0X2345, 0X2407, 0X24C7, \
    0X2588, 0X2648, 0X2707, 0X27C7, 0X2886, 0X2944, 0X2A02, 0X2AC0, \
    0X2B7D, 0X2C3A, 0X2CF7, 0X2DB3, 0X2E6E, 0X2F29, 0X2FE4, 0X309E, \
    0X3158, 0X3211, 0X32CA, 0X3382, 0X343A, 0X34F2, 0X35A8, 0X365F, \
    0X3714, 0X37CA, 0X387E, 0X3932, 0X39E6, 0X3A99, 0X3B4C, 0X3BFD, \
    0X3CAF, 0X3D60, 0X3E10, 0X3EBF, 0X3F6E, 0X401D, 0X40CA, 0X4177, \
    0X4224, 0X42D0, 0X437B, 0X4425, 0X44CF, 0X4578, 0X4621, 0X46C9, \
    0X4770, 0X4816, 0X48BC, 0X4961, 0X4A06, 0X4AA9, 0X4B4C, 0X4BEF, \
    0X4C90, 0X4D31, 0X4DD1, 0X4E70, 0X4F0F, 0X4FAC, 0X5049, 0X50E5, \
    0X5181, 0X521C, 0X52B5, 0X534E, 0X53E7, 0X547E, 0X5515, 0X55AB, \
    0X5640, 0X56D4, 0X5767, 0X57F9, 0X588B, 0X591C, 0X59AC, 0X5A3B, \
    0X5AC9, 0X5B56, 0X5BE3, 0X5C6E, 0X5CF9, 0X5D83, 0X5E0B, 0X5E93, \
    0X5F1A, 0X5FA0, 0X6026, 0X60AA, 0X612D, 0X61B0, 0X6231, 0X62B2, \
    0X6331, 0X63B0, 0X642D, 0X64AA, 0X6526, 0X65A0, 0X661A, 0X6693, \
    0X670B, 0X6782, 0X67F7, 0X686C, 0X68E0, 0X6953, 0X69C4, 0X6A35, \
    0X6AA5, 0X6B13, 0X6B81, 0X6BEE, 0X6C59, 0X6CC4, 0X6D2D, 0X6D96, \
    0X6DFD, 0X6E63, 0X6EC9, 0X6F2D, 0X6F90, 0X6FF2, 0X7053, 0X70B3, \
    0X7112, 0X716F, 0X71CC, 0X7227, 0X7282, 0X72DB, 0X7333, 0X738A, \
    0X73E0, 0X7435, 0X7489, 0X74DB, 0X752D, 0X757D, 0X75CC, 0X761B, \
    0X7668, 0X76B3, 0X76FE, 0X7747, 0X7790, 0X77D7, 0X781D, 0X7862, \
    0X78A6, 0X78E8, 0X792A, 0X796A, 0X79A9, 0X79E7, 0X7A24, 0X7A5F, \
    0X7A9A, 0X7AD3, 0X7B0B, 0X7B42, 0X7B77, 0X7BAC, 0X7BDF, 0X7C11, \
    0X7C42, 0X7C71, 0X7CA0, 0X7CCD, 0X7CF9, 0X7D24, 0X7D4E, 0X7D76, \
    0X7D9D, 0X7DC3, 0X7DE8, 0X7E0C, 0X7E2E, 0X7E4F, 0X7E6F, 0X7E8E, \
    0X7EAB, 0X7EC8, 0X7EE3, 0X7EFD, 0X7F15, 0X7F2D, 0X7F43, 0X7F58, \
    0X7F6B, 0X7F7E, 0X7F8F, 0X7F9F, 0X7FAE, 0X7FBC, 0X7FC8, 0X7FD3, \
    0X7FDD, 0X7FE5, 0X7FED, 0X7FF3, 0X7FF8, 0X7FFC, 0X7FFE, 0X7FFF }
#elif (BASE_MATH_SINCOS_MIDDLE_TABLE == BASE_CFG_DISABLE)
#define TRIGONOMETRIC_MAPPING_TABLE { \
    0x0000, 0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057F, \
    0x0647, 0x0710, 0x07D9, 0x08A2, 0x096A, 0x0A33, 0x0AFB, 0x0BC3, \
    0x0C8B, 0x0D53, 0x0E1B, 0x0EE3, 0x0FAB, 0x1072, 0x1139, 0x1201, \
    0x12C8, 0x138E, 0x1455, 0x151B, 0x15E2, 0x16A8, 0x176D, 0x1833, \
    0x18F8, 0x19BD, 0x1A82, 0x1B47, 0x1C0B, 0x1CCF, 0x1D93, 0x1E56, \
    0x1F19, 0x1FDC, 0x209F, 0x2161, 0x2223, 0x22E5, 0x23A6, 0x2467, \
    0x2528, 0x25E8, 0x26A8, 0x2767, 0x2826, 0x28E5, 0x29A3, 0x2A61, \
    0x2B1F, 0x2BDC, 0x2C98, 0x2D55, 0x2E11, 0x2ECC, 0x2F87, 0x3041, \
    0x30FB, 0x31B5, 0x326E, 0x3326, 0x33DE, 0x3496, 0x354D, 0x3604, \
    0x36BA, 0x376F, 0x3824, 0x38D8, 0x398C, 0x3A40, 0x3AF2, 0x3BA5, \
    0x3C56, 0x3D07, 0x3DB8, 0x3E68, 0x3F17, 0x3FC5, 0x4073, 0x4121, \
    0x41CE, 0x427A, 0x4325, 0x43D0, 0x447A, 0x4524, 0x45CD, 0x4675, \
    0x471C, 0x47C3, 0x4869, 0x490F, 0x49B4, 0x4A58, 0x4AFB, 0x4B9E, \
    0x4C3F, 0x4CE1, 0x4D81, 0x4E21, 0x4EBF, 0x4F5E, 0x4FFB, 0x5097, \
    0x5133, 0x51CE, 0x5269, 0x5302, 0x539B, 0x5433, 0x54CA, 0x5560, \
    0x55F5, 0x568A, 0x571D, 0x57B0, 0x5842, 0x58D4, 0x5964, 0x59F3, \
    0x5A82, 0x5B10, 0x5B9D, 0x5C29, 0x5CB4, 0x5D3E, 0x5DC7, 0x5E50, \
    0x5ED7, 0x5F5E, 0x5FE3, 0x6068, 0x60EC, 0x616F, 0x61F1, 0x6271, \
    0x62F2, 0x6371, 0x63EF, 0x646C, 0x64E8, 0x6563, 0x65DD, 0x6657, \
    0x66CF, 0x6746, 0x67BD, 0x6832, 0x68A6, 0x6919, 0x698C, 0x69FD, \
    0x6A6D, 0x6ADC, 0x6B4A, 0x6BB8, 0x6C24, 0x6C8F, 0x6CF9, 0x6D62, \
    0x6DCA, 0x6E30, 0x6E96, 0x6EFB, 0x6F5F, 0x6FC1, 0x7023, 0x7083, \
    0x70E2, 0x7141, 0x719E, 0x71FA, 0x7255, 0x72AF, 0x7307, 0x735F, \
    0x73B5, 0x740B, 0x745F, 0x74B2, 0x7504, 0x7555, 0x75A5, 0x75F4, \
    0x7641, 0x768E, 0x76D9, 0x7723, 0x776C, 0x77B4, 0x77FA, 0x7840, \
    0x7884, 0x78C7, 0x7909, 0x794A, 0x798A, 0x79C8, 0x7A05, 0x7A42, \
    0x7A7D, 0x7AB6, 0x7AEF, 0x7B26, 0x7B5D, 0x7B92, 0x7BC5, 0x7BF8, \
    0x7C29, 0x7C5A, 0x7C89, 0x7CB7, 0x7CE3, 0x7D0F, 0x7D39, 0x7D62, \
    0x7D8A, 0x7DB0, 0x7DD6, 0x7DFA, 0x7E1D, 0x7E3F, 0x7E5F, 0x7E7F, \
    0x7E9D, 0x7EBA, 0x7ED5, 0x7EF0, 0x7F09, 0x7F21, 0x7F38, 0x7F4D, \
    0x7F62, 0x7F75, 0x7F87, 0x7F97, 0x7FA7, 0x7FB5, 0x7FC2, 0x7FCE, \
    0x7FD8, 0x7FE1, 0x7FE9, 0x7FF0, 0x7FF6, 0x7FFA, 0x7FFD, 0x7FFF }
#endif

#define BASE_MATH_SIN_COS_MASK          0x0300u /**< All mask values of sincos */
#define BASE_MATH_ANGLED0_90            0x0200u /**< Mask value of sincos ranging from 0 to 90 degrees */
#define BASE_MATH_ANGLED90_180          0x0300u /**< Mask value of sincos ranging from 90 to 180 degrees */
#define BASE_MATH_ANGLED180_270         0x0000u /**< Mask value of sincos ranging from 180 to 270 degrees */
#define BASE_MATH_ANGLED270_360         0x0100u /**< Mask value of sincos ranging from 270 to 360 degrees */
#define BASE_MATH_PAI                   3.141592653
#define BASE_MATH_FACTORIAL3_RECIPROCAL 0.166666667  /**< 1/6. */
#define BASE_MATH_FACTORIAL5_RECIPROCAL 0.008333333  /**< 1/120. */
#define BASE_MATH_FACTORIAL7_RECIPROCAL 0.000198413  /**< 1/5040. */
#define BASE_MATH_ANGLE90               90
#define BASE_MATH_ANGLE180              180
#define BASE_MATH_ANGLE180_RECIPROCAL   0.005555556  /**< 1/180. */
#define BASE_MATH_ANGLE270              270
#define BASE_MATH_ANGLE360              360

#define BASE_DEFINE_MAPPING_TABLE_SIZE 255
/** Value to be added to convert a signed 16-bit value to an unsigned 16-bit value. */
#define BASE_DEFINE_INT16_ADDITIONS_VAL 32768

#define BASE_DEFINE_DIV_SQRT3 (int)0x49E6 /**< 1/sqrt(3) = 0.5773315 in q15 format. */

/* Private variables --------------------------------------------------------- */
const short g_triFunMappingTable[] = TRIGONOMETRIC_MAPPING_TABLE; /**< trigonometric look-up table. */

/**
  * @brief Calculate the value of the input angle by looking up the table. Data in Q15 format.
  * @param angle: Angle value to be calculated.
  * @retval Calculation result in BASE_MathTypeSinCos Structure.
  */
BASE_MathTypeSinCos BASE_MATH_GetSinCos(short angle)
{
    BASE_MathTypeSinCos ret = {0};
    unsigned short uhindex;

    /* Move the zero to ensure that the mapping result is positive. */
    uhindex = (unsigned short)((int)BASE_DEFINE_INT16_ADDITIONS_VAL + (int)angle);

    /* Shift right by 6 bits. */
    uhindex /= (unsigned short)64; /* 64:Reserved 10-bit precision. */

    switch ((unsigned short)(uhindex) & BASE_MATH_SIN_COS_MASK) {
        case BASE_MATH_ANGLED0_90: /* 0 ~ 90° */
            ret.sin = g_triFunMappingTable[(unsigned char)(uhindex)];
            ret.cos = g_triFunMappingTable[(unsigned char)(BASE_DEFINE_MAPPING_TABLE_SIZE - (unsigned char)(uhindex))];
            break;

        case BASE_MATH_ANGLED90_180: /* 90 ~ 180° */
            ret.sin = g_triFunMappingTable[(unsigned char)(BASE_DEFINE_MAPPING_TABLE_SIZE - (unsigned char)(uhindex))];
            ret.cos = -g_triFunMappingTable[(unsigned char)(uhindex)];
            break;

        case BASE_MATH_ANGLED180_270: /* 180 ~ 270° */
            ret.sin = -g_triFunMappingTable[(unsigned char)(uhindex)];
            ret.cos = -g_triFunMappingTable[(unsigned char)(BASE_DEFINE_MAPPING_TABLE_SIZE - (unsigned char)(uhindex))];
            break;

        case BASE_MATH_ANGLED270_360: /* 270 ~ 360° */
            ret.sin = -g_triFunMappingTable[(unsigned char)(BASE_DEFINE_MAPPING_TABLE_SIZE - (unsigned char)(uhindex))];
            ret.cos = g_triFunMappingTable[(unsigned char)(uhindex)];
            break;

        default:
            break;
    }

    return ret;
}

/**
  * @brief Using Taylor Expansion to Calculate Sin Values in 90 Degrees.
  * @param angle Angle value to be calculated. Note: 0 <= angle <= 90.
  * @retval float Calculated sin value.
  */
static float BASE_MATH_CalSinIn90(float angle)
{
    float radian = angle * BASE_MATH_PAI * BASE_MATH_ANGLE180_RECIPROCAL;
    float radian3 = radian * radian * radian; /* power(3) */
    float radian5 = radian3 * radian * radian;
    float radian7 = radian5 * radian * radian; /* power(7) */
    /* Using Taylor Expansion to Calculate Sin Values in 90 Degrees. */
    return (radian - radian3 * BASE_MATH_FACTORIAL3_RECIPROCAL + radian5 * BASE_MATH_FACTORIAL5_RECIPROCAL \
    - radian7 * BASE_MATH_FACTORIAL7_RECIPROCAL);
}

/**
  * @brief Using Taylor Expansion to Calculate Sin Values for Any Angle.
  * @param angle Angle value to be calculated.
  * @retval float Calculated sin value.
  */
float BASE_MATH_GetSin(float angle)
{
    float angleIn360;
    angleIn360 = (int)angle % BASE_MATH_ANGLE360 + angle - (int)angle;
    if (angleIn360 < 0) {
        angleIn360 = angleIn360 + BASE_MATH_ANGLE360;
    }
    if (angleIn360 < BASE_MATH_ANGLE90) { /* 0 ~ 90° */
        return BASE_MATH_CalSinIn90(angleIn360);
    }
    if (angleIn360 < BASE_MATH_ANGLE180) { /* 90 ~ 180° */
        return BASE_MATH_CalSinIn90(BASE_MATH_ANGLE180 - angleIn360);
    }
    if (angleIn360 < BASE_MATH_ANGLE270) { /* 180 ~ 270° */
        return -BASE_MATH_CalSinIn90(angleIn360 - BASE_MATH_ANGLE180);
    }
    return -BASE_MATH_CalSinIn90(BASE_MATH_ANGLE360 - angleIn360); /* 270 ~ 360° */
}

/**
  * @brief Using Taylor Expansion to Calculate Sin Values for Any Angle.
  * @param angle Angle value to be calculated.
  * @retval float Calculated sin value.
  */
float BASE_MATH_GetCos(float angle)
{
    float angleIn360;
    angleIn360 = (int)angle % BASE_MATH_ANGLE360 + angle - (int)angle;
    if (angleIn360 < 0) {
        angleIn360 = angleIn360 + BASE_MATH_ANGLE360;
    }
    if (angleIn360 < BASE_MATH_ANGLE90) { /* 0 ~ 90° */
        return BASE_MATH_CalSinIn90(BASE_MATH_ANGLE90 - angleIn360);
    }
    if (angleIn360 < BASE_MATH_ANGLE180) { /* 90 ~ 180° */
        return -BASE_MATH_CalSinIn90(angleIn360 - BASE_MATH_ANGLE90);
    }
    if (angleIn360 < BASE_MATH_ANGLE270) { /* 180 ~ 270° */
        return -BASE_MATH_CalSinIn90(BASE_MATH_ANGLE270 - angleIn360);
    }
    return BASE_MATH_CalSinIn90(angleIn360 - BASE_MATH_ANGLE270); /* 270 ~ 360° */
}

/**
  * @brief Using newton iteration method to realize sqrt.
  * @param x Value to be squared.
  * @retval float Value after square.
  */
float BASE_MATH_Sqrt(const float x)
{
    BASE_FUNC_ASSERT_PARAM(x >= FLT_EPSILON);
    const float xHalf = 0.5f * x; /* 0.5f : coefficients. */

    union {
        float x;
        unsigned int i;
    } u;
    u.x = x;
    u.i = 0x5f3759df - (u.i >> 1); /* 0x5f3759df : Magic numbers for sqrt. */
    return x * u.x * (1.5f - xHalf * u.x * u.x); /* 1.5f : coefficients. */
}

/**
  * @brief Compute x to the n power.
  * @param x Cardinality to be calculated.
  * @param n Power exponent to be calculated.
  * @retval float Calculated value.
  */
float BASE_MATH_Pow(float x, int n)
{
    /* check x not equal zero */
    if (x > -FLT_EPSILON && x < FLT_EPSILON) {
        return 0.0f;
    }
    float value = x;
    int power = n;
    float res = 1.0;
    if (power < 0) {
        value = 1 / value;
        power = -power;
    }
    /* power multiplication */
    for (unsigned int i = 0; i < (unsigned int)power; ++i) {
        res *= value;
    }
    return res;
}

/**
  * @brief This function performs Clarke conversion. Data in Q15 format. The conversion formula is as follows:
  * alpha = a;
  * beta  = 1 / sqrt3 * a + 2 / sqrt3 *b.
  * @param input Current values of a\b items.
  * @retval BASE_MathTypeAlphaBeta Conversion result in BASE_MathTypeAlphaBeta Structure.
  */
BASE_MathTypeAlphaBeta BASE_MATH_Clarke(BASE_MathTypeAB input)
{
    BASE_MathTypeAlphaBeta ret;
    int aDivSort3, bDivSort3, betaTmp32;

    /* qIalpha = qIas. */
    ret.alpha = input.a;

    aDivSort3 = BASE_DEFINE_DIV_SQRT3 * (int)input.a;

    bDivSort3 = BASE_DEFINE_DIV_SQRT3 * (int)input.b;

    /* qIbeta = (2*qIbs+qIas)/sqrt(3). */
    /* Because BASE_DEFINE_DIV_SQRT3 is in the Q15 format, divide it by 32768 to ensure that the result is correct. */
    betaTmp32 = (aDivSort3 + bDivSort3 + bDivSort3) >> 15; /* 15:Move 15 bits to the right, keep Q15 format. */

    /* Check saturation of Ibeta */
    if (betaTmp32 > INT16_MAX) {
        ret.beta = INT16_MAX;
    } else if (betaTmp32 < INT16_MIN) {
        ret.beta = INT16_MIN;
    } else {
        ret.beta = (short)(betaTmp32);
    }

    return ret;
}

/**
  * @brief This function performs Park coordinate conversion. Data in Q15 format. The conversion formula is as follows:
  * id =  alpha * cos(theta) + beta * sin(theta);
  * iq = -alpha * sin(theta) + beta * cos(theta).
  * @param input stator values alpha and beta in BASE_MathTypeAlphaBeta format.
  * @param theta rotating frame angular position.
  * @retval BASE_MathTypeQD Conversion result in BASE_MathTypeQD Structure.
  */
BASE_MathTypeQD BASE_MATH_Park(BASE_MathTypeAlphaBeta input, short theta)
{
    BASE_MathTypeQD ret;
    BASE_MathTypeSinCos thetaSinCos;
    int d1, d2, q1, q2, tmp32;

    thetaSinCos = BASE_MATH_GetSinCos(theta);

    /* No overflow guaranteed. */
    d1 = input.alpha * (int)thetaSinCos.cos;

    /* No overflow guaranteed. */
    d2 = input.beta * (int)thetaSinCos.sin;

    /* Id component in Q1.15 Format. */
    tmp32 = (d1 + d2) >> 15; /* 15:Move 15 bits to the right, keep Q15 format. */

    /* Check saturation of Id. */
    if (tmp32 > INT16_MAX) {
        ret.d = INT16_MAX;
    } else if (tmp32 < INT16_MIN) {
        ret.d = INT16_MIN;
    } else {
        ret.d = (short)(tmp32);
    }

    /* No overflow guaranteed. */
    q1 = input.alpha * (int)thetaSinCos.sin;

    /* No overflow guaranteed. */
    q2 = input.beta * (int)thetaSinCos.cos;

    /* Iq component in Q1.15 Format. */
    tmp32 = (q2 - q1) >> 15; /* 15:Move 15 bits to the right, keep Q15 format. */

    /* Check saturation of Iq. */
    if (tmp32 > INT16_MAX) {
        ret.q = INT16_MAX;
    } else if (tmp32 < INT16_MIN) {
        ret.q = INT16_MIN;
    } else {
        ret.q = (short)(tmp32);
    }

    return ret;
}

/**
  * @brief This function performs Reverse Park coordinate conversion. Data in Q15 format. The conversion formula is as
  * follows: alpha = d * cos(theta) - q * sin(theta);
  *          beta  = d * sin(theta) + q * cos(theta).
  * @param input stator voltage Vq and Vd in BASE_MathTypeQD format.
  * @param theta rotating frame angular position.
  * @retval BASE_MathTypeAlphaBeta Conversion result in BASE_MathTypeAlphaBeta Structure.
  */
BASE_MathTypeAlphaBeta BASE_MATH_RevPark(BASE_MathTypeQD input, short theta)
{
    int alpha1, alpha2, beta1, beta2;
    BASE_MathTypeSinCos thetaSinCos;
    BASE_MathTypeAlphaBeta ret;

    thetaSinCos = BASE_MATH_GetSinCos(theta);

    /* No overflow guaranteed. */
    alpha1 = input.q * (int)thetaSinCos.sin;
    alpha2 = input.d * (int)thetaSinCos.cos;

    ret.alpha = (short)((alpha2 - alpha1) >> 15); /* 15:Move 15 bits to the right, keep Q15 format. */

    beta1 = input.q * (int)thetaSinCos.cos;
    beta2 = input.d * (int)thetaSinCos.sin;

    ret.beta = (short)((beta1 + beta2) >> 15); /* 15:Move 15 bits to the right, keep Q15 format. */

    return ret;
}
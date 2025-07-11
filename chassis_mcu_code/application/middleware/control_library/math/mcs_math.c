/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_math.c
  * @author    MCU Algorithm Team
  * @brief     This file provides common math functions including trigonometric, coordinate transformation,
  *            square root math calculation.
  */

#include "mcs_math.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/* Macro definitions --------------------------------------------------------------------------- */
#define SIN_TABLE \
    { \
        0, 51, 101, 151, 202, 252, 302, 352, 403, 453, 503, 553, 604, 654, 704, 754, 805, 855, 905, 955, 1006, 1056, \
        1106, 1156, 1207, 1257, 1307, 1357, 1407, 1458, 1508, 1558, 1608, 1659, 1709, 1759, 1809, 1859, 1909, \
        1960, 2010, 2060, 2110, 2160, 2210, 2261, 2311, 2361, 2411, 2461, 2511, 2561, 2611, 2662, 2712, 2762, \
        2812, 2862, 2912, 2962, 3012, 3062, 3112, 3162, 3212, 3262, 3312, 3362, 3412, 3462, 3512, 3562, 3612, \
        3662, 3712, 3762, 3812, 3862, 3912, 3962, 4012, 4061, 4111, 4161, 4211, 4261, 4311, 4360, 4410, 4460, \
        4510, 4560, 4609, 4659, 4709, 4759, 4808, 4858, 4908, 4958, 5007, 5057, 5107, 5156, 5206, 5255, 5305, \
        5355, 5404, 5454, 5503, 5553, 5602, 5652, 5701, 5751, 5800, 5850, 5899, 5949, 5998, 6048, 6097, 6146, \
        6196, 6245, 6294, 6344, 6393, 6442, 6492, 6541, 6590, 6639, 6689, 6738, 6787, 6836, 6885, 6934, 6983, \
        7033, 7082, 7131, 7180, 7229, 7278, 7327, 7376, 7425, 7474, 7523, 7572, 7620, 7669, 7718, 7767, 7816, \
        7865, 7913, 7962, 8011, 8060, 8108, 8157, 8206, 8254, 8303, 8352, 8400, 8449, 8497, 8546, 8594, 8643, \
        8691, 8740, 8788, 8837, 8885, 8933, 8982, 9030, 9078, 9127, 9175, 9223, 9271, 9320, 9368, 9416, 9464, \
        9512, 9560, 9608, 9656, 9704, 9752, 9800, 9848, 9896, 9944, 9992, 10040, 10088, 10136, 10183, 10231, \
        10279, 10327, 10374, 10422, 10470, 10517, 10565, 10612, 10660, 10707, 10755, 10802, 10850, 10897, 10945, \
        10992, 11039, 11087, 11134, 11181, 11228, 11276, 11323, 11370, 11417, 11464, 11511, 11558, 11605, 11652, \
        11699, 11746, 11793, 11840, 11887, 11934, 11981, 12027, 12074, 12121, 12167, 12214, 12261, 12307, 12354, \
        12400, 12447, 12493, 12540, 12586, 12633, 12679, 12725, 12772, 12818, 12864, 12910, 12957, 13003, 13049, \
        13095, 13141, 13187, 13233, 13279, 13325, 13371, 13417, 13463, 13508, 13554, 13600, 13646, 13691, 13737, \
        13783, 13828, 13874, 13919, 13965, 14010, 14056, 14101, 14146, 14192, 14237, 14282, 14327, 14373, 14418, \
        14463, 14508, 14553, 14598, 14643, 14688, 14733, 14778, 14823, 14867, 14912, 14957, 15002, 15046, 15091, \
        15136, 15180, 15225, 15269, 15314, 15358, 15402, 15447, 15491, 15535, 15580, 15624, 15668, 15712, 15756, \
        15800, 15844, 15888, 15932, 15976, 16020, 16064, 16108, 16151, 16195, 16239, 16282, 16326, 16369, 16413, \
        16456, 16500, 16543, 16587, 16630, 16673, 16717, 16760, 16803, 16846, 16889, 16932, 16975, 17018, 17061, \
        17104, 17147, 17190, 17233, 17275, 17318, 17361, 17403, 17446, 17488, 17531, 17573, 17616, 17658, 17700, \
        17743, 17785, 17827, 17869, 17911, 17953, 17995, 18037, 18079, 18121, 18163, 18205, 18247, 18288, 18330, \
        18372, 18413, 18455, 18496, 18538, 18579, 18621, 18662, 18703, 18745, 18786, 18827, 18868, 18909, 18950, \
        18991, 19032, 19073, 19114, 19155, 19195, 19236, 19277, 19317, 19358, 19398, 19439, 19479, 19520, 19560, \
        19600, 19641, 19681, 19721, 19761, 19801, 19841, 19881, 19921, 19961, 20001, 20041, 20080, 20120, 20160, \
        20199, 20239, 20278, 20318, 20357, 20397, 20436, 20475, 20514, 20554, 20593, 20632, 20671, 20710, 20749, \
        20788, 20826, 20865, 20904, 20943, 20981, 21020, 21058, 21097, 21135, 21174, 21212, 21250, 21289, 21327, \
        21365, 21403, 21441, 21479, 21517, 21555, 21593, 21630, 21668, 21706, 21744, 21781, 21819, 21856, 21894, \
        21931, 21968, 22005, 22043, 22080, 22117, 22154, 22191, 22228, 22265, 22302, 22339, 22375, 22412, 22449, \
        22485, 22522, 22558, 22595, 22631, 22667, 22704, 22740, 22776, 22812, 22848, 22884, 22920, 22956, 22992, \
        23028, 23063, 23099, 23135, 23170, 23206, 23241, 23277, 23312, 23347, 23383, 23418, 23453, 23488, 23523, \
        23558, 23593, 23628, 23662, 23697, 23732, 23767, 23801, 23836, 23870, 23904, 23939, 23973, 24007, 24042, \
        24076, 24110, 24144, 24178, 24212, 24245, 24279, 24313, 24347, 24380, 24414, 24447, 24481, 24514, 24547, \
        24581, 24614, 24647, 24680, 24713, 24746, 24779, 24812, 24845, 24878, 24910, 24943, 24975, 25008, 25040, \
        25073, 25105, 25137, 25170, 25202, 25234, 25266, 25298, 25330, 25362, 25393, 25425, 25457, 25488, 25520, \
        25551, 25583, 25614, 25646, 25677, 25708, 25739, 25770, 25801, 25832, 25863, 25894, 25925, 25955, 25986, \
        26017, 26047, 26078, 26108, 26138, 26169, 26199, 26229, 26259, 26289, 26319, 26349, 26379, 26409, 26438, \
        26468, 26498, 26527, 26557, 26586, 26616, 26645, 26674, 26703, 26732, 26761, 26790, 26819, 26848, 26877, \
        26906, 26934, 26963, 26991, 27020, 27048, 27077, 27105, 27133, 27161, 27189, 27217, 27245, 27273, 27301, \
        27329, 27356, 27384, 27412, 27439, 27467, 27494, 27521, 27549, 27576, 27603, 27630, 27657, 27684, 27711, \
        27737, 27764, 27791, 27817, 27844, 27870, 27897, 27923, 27949, 27976, 28002, 28028, 28054, 28080, 28106, \
        28132, 28157, 28183, 28209, 28234, 28260, 28285, 28310, 28336, 28361, 28386, 28411, 28436, 28461, 28486, \
        28511, 28535, 28560, 28585, 28609, 28634, 28658, 28682, 28707, 28731, 28755, 28779, 28803, 28827, 28851, \
        28875, 28898, 28922, 28946, 28969, 28993, 29016, 29039, 29063, 29086, 29109, 29132, 29155, 29178, 29201, \
        29223, 29246, 29269, 29291, 29314, 29336, 29359, 29381, 29403, 29425, 29447, 29469, 29491, 29513, 29535, \
        29557, 29578, 29600, 29622, 29643, 29664, 29686, 29707, 29728, 29749, 29770, 29791, 29812, 29833, 29854, \
        29874, 29895, 29916, 29936, 29956, 29977, 29997, 30017, 30037, 30057, 30077, 30097, 30117, 30137, 30157, \
        30176, 30196, 30215, 30235, 30254, 30273, 30292, 30312, 30331, 30350, 30369, 30387, 30406, 30425, 30443, \
        30462, 30481, 30499, 30517, 30536, 30554, 30572, 30590, 30608, 30626, 30644, 30661, 30679, 30697, 30714, \
        30732, 30749, 30767, 30784, 30801, 30818, 30835, 30852, 30869, 30886, 30903, 30919, 30936, 30952, 30969, \
        30985, 31002, 31018, 31034, 31050, 31066, 31082, 31098, 31114, 31129, 31145, 31161, 31176, 31192, 31207, \
        31222, 31237, 31253, 31268, 31283, 31298, 31312, 31327, 31342, 31357, 31371, 31386, 31400, 31414, 31429, \
        31443, 31457, 31471, 31485, 31499, 31513, 31526, 31540, 31554, 31567, 31581, 31594, 31607, 31620, 31634, \
        31647, 31660, 31673, 31685, 31698, 31711, 31724, 31736, 31749, 31761, 31773, 31786, 31798, 31810, 31822, \
        31834, 31846, 31857, 31869, 31881, 31892, 31904, 31915, 31927, 31938, 31949, 31960, 31971, 31982, 31993, \
        32004, 32015, 32025, 32036, 32047, 32057, 32067, 32078, 32088, 32098, 32108, 32118, 32128, 32138, 32148, \
        32157, 32167, 32177, 32186, 32195, 32205, 32214, 32223, 32232, 32241, 32250, 32259, 32268, 32276, 32285, \
        32294, 32302, 32311, 32319, 32327, 32335, 32343, 32351, 32359, 32367, 32375, 32383, 32390, 32398, 32405, \
        32413, 32420, 32427, 32435, 32442, 32449, 32456, 32463, 32469, 32476, 32483, 32489, 32496, 32502, 32509, \
        32515, 32521, 32527, 32533, 32539, 32545, 32551, 32557, 32562, 32568, 32573, 32579, 32584, 32589, 32595, \
        32600, 32605, 32610, 32615, 32619, 32624, 32629, 32633, 32638, 32642, 32647, 32651, 32655, 32659, 32663, \
        32667, 32671, 32675, 32679, 32682, 32686, 32689, 32693, 32696, 32700, 32703, 32706, 32709, 32712, 32715, \
        32718, 32720, 32723, 32726, 32728, 32730, 32733, 32735, 32737, 32739, 32741, 32743, 32745, 32747, 32749, \
        32751, 32752, 32754, 32755, 32756, 32758, 32759, 32760, 32761, 32762, 32763, 32764, 32764, 32765, 32766, \
        32766, 32767, 32767, 32767, 32767, 32767 \
    }

const float atanInBottom[50] = { 0.0f, 0.102040816326531f, 0.204081632653061f, 0.306122448979592f, \
                                 0.408163265306122f, 0.510204081632653f, 0.612244897959184f, 0.714285714285714f, \
                                 0.816326530612245f, 0.918367346938776f, 1.02040816326531f, 1.12244897959184f, \
                                 1.22448979591837f, 1.32653061224490f, 1.42857142857143f, 1.53061224489796f, \
                                 1.63265306122449f, 1.73469387755102f, 1.83673469387755f, 1.93877551020408f, \
                                 2.04081632653061f, 2.14285714285714f, 2.24489795918367f, 2.34693877551020f, \
                                 2.44897959183673f, 2.55102040816327f, 2.65306122448980f, 2.75510204081633f, \
                                 2.85714285714286f, 2.95918367346939f, 3.06122448979592f, 3.16326530612245f, \
                                 3.26530612244898f, 3.36734693877551f, 3.46938775510204f, 3.57142857142857f, \
                                 3.67346938775510f, 3.77551020408163f, 3.87755102040816f, 3.97959183673469f, \
                                 4.08163265306123f, 4.18367346938776f, 4.28571428571429f, 4.38775510204082f, \
                                 4.48979591836735f, 4.59183673469388f, 4.69387755102041f, 4.79591836734694f, \
                                 4.89795918367347f, 5.0f};
const float atanValBottom[50] = { 0.0f, 0.101688851763077f, 0.201317108374641f, 0.297064212341043f, \
                                  0.387523805780279f, 0.471777511180750f, 0.549374484771551f, 0.620249485982822f, \
                                  0.684617164312781f, 0.742870628777664f, 0.795498829982770f, 0.843026590874922f, \
                                  0.885975080852296f, 0.924838220488786f, 0.960070362405688f, 0.992081381881698f, \
                                  1.02123631326852f, 1.04785756322372f, 1.07222842115668f, 1.09459707572452f, \
                                  1.11518067358367f, 1.13416916698136f, 1.15172882709508f, 1.16800537775525f, \
                                  1.18312674842090f, 1.19720546875916f, 1.21034073815249f, 1.22262020713844f, \
                                  1.23412150740817f, 1.24491356451280f, 1.25505772401419f, 1.26460871813527f, \
                                  1.27361549637858f, 1.28212194027307f, 1.29016747945525f, 1.29778762370819f, \
                                  1.30501442335451f, 1.31187686849742f, 1.31840123598843f, 1.32461139163550f, \
                                  1.33052905401396f, 1.33617402527335f, 1.34156439351790f, 1.34671671065198f, \
                                  1.35164614900430f, 1.35636663955779f, 1.36089099420126f, 1.36523101407236f, \
                                  1.36939758576738f, 1.37340076694502f};
const float atanInMid[25] = { 5.0f, 5.625f, 6.25f, 6.875f, 7.5f, \
                              8.125f, 8.75f, 9.375f, 10.0f, 10.625f, \
                              11.25f, 11.875f, 12.5f, 13.125f, 13.75f, \
                              14.375f, 15.0f, 15.625f, 16.25f, 16.875f, \
                              17.5f, 18.125f, 18.75f, 19.375f, 20.0f};
const float atanValMid[25] = { 1.373400766945016f, 1.394856701342369f, 1.41214106460850f, 1.42635474842025f, \
                               1.43824479449822f, 1.44833526937756f, 1.45700431965119f, 1.46453146390382f, \
                               1.47112767430373f, 1.47695511416556f, 1.48214044492746f, 1.48678401498740f, \
                               1.49096634108266f, 1.49475276751578f, 1.49819687306440f, 1.50134300079957f, \
                               1.50422816301907f, 1.50688349400616f, 1.50933537091213f, 1.51160628786678f, \
                               1.51371554438863f, 1.51567979250081f, 1.51751347523520f, 1.51922918085206f, \
                               1.52083793107295f};
const float atanInTop[10] = { 20.0f, 128.888888888889f, 237.777777777778f, 346.666666666667f, \
                              455.555555555556f, 564.444444444445f, 673.333333333333f, 782.222222222222f, \
                              891.111111111111f, 1000.0f};
const float atanValTop[10] = { 1.52083793107295f, 1.56303786177943f, 1.56659074411305f, 1.56791171941121f, \
                               1.56860120836944f, 1.56902467510518f, 1.56931117937196f, 1.56951791840043f, \
                               1.56967413275225f, 1.56979632712823f};

#define SIN_MASK 0x0C00
#define U0_90 0x0800
#define U90_180 0x0C00
#define U180_270 0x0000
#define U270_360 0x0400
#define SIN_TAB_LEN 0x03FF
#define Q15_BASE    32768
#define ANGLE_TO_INDEX_SHIFT  4

#define  ATAN_INPUTVALUE_MIN   5.0f
#define  ATAN_INPUTVALUE_MID   20.0f
#define  ATAN_INPUTVALUE_MAX   1000.0f

#define MATH_FACTORIAL3INVERSE     0.16666667f     /**< 1 / 6 */
#define MATH_FACTORIAL5INVERSE     0.008333333f    /**< 1 / 120 */
#define MATH_FACTORIAL7INVERSE     0.0001984127f   /**< 1 / 5040 */

/* Private variables --------------------------------------------------------- */
const short g_sinTable[SIN_TAB_LEN + 1] = SIN_TABLE;


/**
  * @brief Using Taylor Expansion to Calculate Sin Values in rad.
  * @param angle Angle value to be calculated.
  * @retval float Calculated sin value.
  */
static float TaylorCalSinIn90(float angle)
{
    float radian = angle;
    float radian3 = radian * radian * radian; /* power(3) */
    float radian5 = radian3 * radian * radian;
    float radian7 = radian5 * radian * radian; /* power(7) */
    /* Using Taylor Expansion to Calculate Sin Values in 90 Degrees. */
    return (radian - radian3 * MATH_FACTORIAL3INVERSE + \
            radian5 * MATH_FACTORIAL5INVERSE - radian7 * MATH_FACTORIAL7INVERSE);
}

/**
  * @brief Using Taylor Expansion to Calculate Sin Values for Any Angle.
  * @param angle Angle value to be calculated.
  * @retval float Calculated sin value.
  */
float GetSin(float angle)
{
    /* limit the data scope to (0 - 2PI) */
    float angleIn2pi = Mod(angle, DOUBLE_PI);
    if (angleIn2pi < 0) {
        angleIn2pi = angleIn2pi + DOUBLE_PI;
    }
    if (angleIn2pi < HALF_PI) { /* 0 ~ 90° */
        return TaylorCalSinIn90(angleIn2pi);
    }
    if (angleIn2pi < ONE_PI) { /* 90 ~ 180° */
        return TaylorCalSinIn90(ONE_PI - angleIn2pi);
    }
    if (angleIn2pi < THREE_PI_DIV_TWO) { /* 180 ~ 270° */
        return -TaylorCalSinIn90(angleIn2pi - ONE_PI);
    }
    return -TaylorCalSinIn90(DOUBLE_PI - angleIn2pi); /* 270 ~ 360° */
}

/**
  * @brief Using Taylor Expansion to Calculate Sin Values for Any Angle.
  * @param angle Angle value to be calculated.
  * @retval float Calculated sin value.
  */
float GetCos(float angle)
{
    /* limit the data scope to (0 - 2PI) */
    float angleIn2pi = Mod(angle, DOUBLE_PI);
    if (angleIn2pi < 0) {
        angleIn2pi = angleIn2pi + DOUBLE_PI;
    }
    if (angleIn2pi < HALF_PI) { /* 0 ~ 90° */
        return TaylorCalSinIn90(HALF_PI - angleIn2pi);
    }
    if (angleIn2pi < ONE_PI) { /* 90 ~ 180° */
        return -TaylorCalSinIn90(angleIn2pi - HALF_PI);
    }
    if (angleIn2pi < THREE_PI_DIV_TWO) { /* 180 ~ 270° */
        return -TaylorCalSinIn90(THREE_PI_DIV_TWO - angleIn2pi);
    }
    return TaylorCalSinIn90(angleIn2pi - THREE_PI_DIV_TWO); /* 270 ~ 360° */
}


/**
  * @brief  Calculate sine and cosine function of the input angle.
  * @param  val: Output result, which contain the calculated sin, cos value.
  * @param  angle: The input parameter angle (rad).
  * @retval None.
  */
void TrigCalc(TrigVal *val, float angle)
{
    MCS_ASSERT_PARAM(val != NULL);
    val->sin = GetSin(angle);
    val->cos = GetCos(angle);
}

/**
  * @brief  Park transformation: transforms stator values alpha and beta, which
  *         belong to a stationary albe reference frame, to a rotor flux
  *         synchronous reference dq frame.
  * @param  albe: Input alpha beta axis value.
  * @param  angle: Input the theta angle (rad).
  * @param  dq: Output DQ axis value.
  * @retval None
  */
void ParkCalc(const AlbeAxis *albe, float angle, DqAxis *dq)
{
    MCS_ASSERT_PARAM(albe != NULL);
    MCS_ASSERT_PARAM(dq != NULL);
    float alpha = albe->alpha;
    float beta = albe->beta;
    TrigVal localTrigVal;
    /* The projection of ia, ib, and ic currents on alpha and beta axes is equivalent to that on d, q axes. */
    TrigCalc(&localTrigVal, angle);
    dq->d = alpha * localTrigVal.cos + beta * localTrigVal.sin;
    dq->q = -alpha * localTrigVal.sin + beta * localTrigVal.cos;
}

/**
  * @brief  Inverse Park transformation: transforms stator values d and q, which
  *         belong to a rotor flux synchronous reference dq frame, to a stationary
  *         albe reference frame.
  * @param  dq: Input DQ axis value.
  * @param  angle: Input the theta angle (rad).
  * @param  albe: Output alpha beta axis value.
  * @retval None
  */
void InvParkCalc(const DqAxis *dq, float angle, AlbeAxis *albe)
{
    MCS_ASSERT_PARAM(dq != NULL);
    MCS_ASSERT_PARAM(albe != NULL);
    float d = dq->d;
    float q = dq->q;
    TrigVal localTrigVal;
    /* Inversely transform the d, q-axis current to alpha ,beta. */
    TrigCalc(&localTrigVal, angle);
    albe->alpha = d * localTrigVal.cos - q * localTrigVal.sin;
    albe->beta = d * localTrigVal.sin + q * localTrigVal.cos;
}

/**
  * @brief Clarke transformation: transforms stationary three-phase quantites to
  *        stationary albe quantites.
  * @param uvw: Clarke struct handle.
  * @param albe: AlbeAxis struct handle used to store the Clarke transform output.
  * @retval None.
  */
void ClarkeCalc(const UvwAxis *uvw, AlbeAxis *albe)
{
    MCS_ASSERT_PARAM(uvw != NULL);
    MCS_ASSERT_PARAM(albe != NULL);
    albe->alpha = uvw->u;
    albe->beta  = ONE_DIV_SQRT3 * (uvw->u + 2.0f * uvw->v);
}

/**
  * @brief This function returns the absolute value of the input value.
  * @param val: The quantity that wants to execute absolute operation.
  * @retval The absolute value of the input value.
  */
float Abs(float val)
{
    return (val >= 0.0f) ? val : (-val);
}

/**
  * @brief Clamp operation.
  * @param val Value that needs to be clamped.
  * @param upperLimit The upper limitation.
  * @param lowerLimit The lower limitation.
  * @retval Clamped value.
  */
float Clamp(float val, float upperLimit, float lowerLimit)
{
    MCS_ASSERT_PARAM(upperLimit > lowerLimit);
    float result;
    /* Clamping Calculation. */
    if (val >= upperLimit) {
        result = upperLimit;
    } else if (val <= lowerLimit) {
        result = lowerLimit;
    } else {
        result = val;
    }
    return result;
}

/**
  * @brief Get bigger value.
  * @param val1 The value to be compared.
  * @param val2 The value to be compared.
  * @retval The greater value.
  */
float Max(float val1, float val2)
{
    return ((val1 >= val2) ? val1 : val2);
}

/**
  * @brief Get smaller value.
  * @param val1 The value to be compared.
  * @param val2 The value to be compared.
  * @retval The smaller value.
  */
float Min(float val1, float val2)
{
    return ((val1 <= val2) ? val1 : val2);
}

/**
  * @brief Fast sqrt calculation using ASM.
  * @param val Float val.
  * @retval Sqrt result.
  */
float Sqrt(float val)
{
    MCS_ASSERT_PARAM(val >= 0.0f);
    float rd = val;

    __asm volatile("fsqrt.s %0, %1" : "=f"(rd) : "f"(val));

    return rd;
}


/**
  * @brief Angle difference calculation.
  * @param angle1 Angle to be substracted.
  * @param angle2 Angle to substract.
  * @retval Angle difference.
  */
float AngleSub(float angle1, float angle2)
{
    /* Calculate the error of the two angle. */
    float err = angle1 - angle2;

    /* If error between -pi to pi, return error without changes. */
    err = Mod(err, DOUBLE_PI);
    if (err > ONE_PI) {
        err -= DOUBLE_PI;
    } else if (err < -ONE_PI) {
        err += DOUBLE_PI;
    }

    return err;
}


/**
  * @brief Dichotomy to find the position of the target value in the array.
  * @param u: Target Value.
  * @param table: Pointer of Array.
  * @param startIndex: Start Index
  * @param maxIndex: Max Index.
  * @retval Target index.
  */

static unsigned short BinSearch(float u, const float *table,
                                unsigned short startIndex,
                                unsigned short maxIndex)
{
    MCS_ASSERT_PARAM(table != NULL);
    //MCS_ASSERT_PARAM(startIndex >= 0);
    //MCS_ASSERT_PARAM(maxIndex >= 0);
    /* The dot to the left of the dichotomy */
    unsigned short iLeft;
    /* The dot to the right of the dichotomy */
    unsigned short iRight;
    /* The point in the middle of the dichotomy */
    unsigned short iMid;
    
    /* Binary Search */
    iMid = startIndex;
    iLeft = 0U;
    iRight = maxIndex;
    while ((unsigned short)(iRight - iLeft) > 1U) {
        if (u < table[iMid]) {
            /* The target value is a bit smaller than the current value on the left */
            iRight = iMid;
        } else {
            /* TThe target value is greater than the current value on the right */
            iLeft = iMid;
        }
        /* Get the next intermediate point */
        iMid = ((unsigned short)(iRight + iLeft)) >> 1;
    }
    return iLeft;
}

/**
  * @brief Dichotomy to find the position of the target value in the array.
  * @param u: Target Value.
  * @param table: Pointer of Array.
  * @param fraction: Poniter ratio value addr.
  * @param maxIndex: Max Index.
  * @retval Target index.
  */
static unsigned short PreLookBinSearch(float u, const float *table,
                                       unsigned short maxIndex,
                                       float *fraction)
{
    MCS_ASSERT_PARAM(table != NULL);
    MCS_ASSERT_PARAM(fraction != NULL);
    /* Dichotomy to find the position of the target value in the array */
    unsigned short index;
    if (u <= table[0U]) {
        /* Less than the minimum value in the table */
        index = 0U;
        *fraction = 0.0f;
    } else if (u < table[maxIndex]) {
        index = BinSearch(u, table, maxIndex >> 1U, maxIndex);
        *fraction = (u - table[index]) / (table[index + 1U] - table[index]);
    } else {
        /* Greater than the minimum value in the table */
        index = maxIndex;
        *fraction = 0.0f;
    }
    return index;
}

/**
  * @brief calculating arc tangent.
  * @param u: Target Value.
  * @retval Arctangent value of U.
  */
static float ATan(float u)
{
    float tmp = Abs(u);
    float frac = 0.0f;
    unsigned short index = 0;
    float y = 0.0f;
    if (tmp >= 0.0f && tmp < ATAN_INPUTVALUE_MIN) {
        index = PreLookBinSearch(tmp, atanInBottom, 49U, &frac); /* atanInBottom Max Index is 49 */
        y = atanValBottom[index] + frac * (atanValBottom[index + 1] - atanValBottom[index]);
    } else if (tmp >= ATAN_INPUTVALUE_MIN && tmp < ATAN_INPUTVALUE_MID) {
        index = PreLookBinSearch(tmp, atanInMid, 24U, &frac);   /* atanInMid Max Index is 24 */
        y = atanValMid[index] + frac * (atanValMid[index + 1] - atanValMid[index]);
    } else if (tmp >= ATAN_INPUTVALUE_MID && tmp < ATAN_INPUTVALUE_MAX) {
        index = PreLookBinSearch(tmp, atanInTop, 9U, &frac);  /* atanInTop Max Index is 9 */
        y = atanValTop[index] + frac * (atanValTop[index + 1] - atanValTop[index]);
    } else {
        y = HALF_PI; /* The input parameter is greater than the maximum radian, The value is PI/2. */
    }
    return (u > 0.0f)? y : (- y);
}


/**
  * @brief modulo operation.
  * @param val1 The value to be modulo.
  * @param val2 The value to modulo.
  * @retval modulo result.
  */
float Mod(float val1, float val2)
{
    MCS_ASSERT_PARAM(val2 > 0.0f);
    
    int temp = (int)(val1 / val2);
    float res = val1 - (float)temp * val2;
    return res;
}


/**
  * @brief Atan2 arctangent calculation.
  * @param x Floating-point value representing the X-axis coordinate.
  * @param y Floating-point value representing the Y-axis coordinate.
  * @retval The atan2 function returns the azimuth from the origin to the point (x, y), that is,
            the angle from the x axis. It can also be understood as the argument of the complex number x+yi.
            The unit of the returned value is radian. The value range is -pi ~ pi.
  */
float Atan2(float x, float y)
{
    float fZero = 0.0f;
    if (x > fZero) {
        return ATan(y / x);
    }
    if (x < fZero && y >= fZero) {
        return ATan(y / x) + ONE_PI;
    }
    if (x < fZero && y < fZero) {
        return ATan(y / x) - ONE_PI;
    }
    /* boundary condition */
    if ((Abs(x) < 0.0001f) && y > fZero) {
        return (HALF_PI);
    }
    if (Abs(x) < 0.0001f && y < fZero) {
        return -(HALF_PI);
    }
    /* default return */
    return fZero;
}

/**
  * @brief Saturation function for dead voltage computing.
  * @param u The current amp of zero crossing point.
  * @param delta Saturated output point.
  * @return Saturation value ([-1.0f, 1.0f]).
  */
float Sat(float u, float delta)
{
    BASE_FUNC_ASSERT_PARAM(delta > 0.0f);
    /* less than -0.1, return -1 */
    if (u < -delta) {
        return -1.0f;
    } else if (u > delta) { /* large than 0.1, return 1 */
        return 1.0f;
    } else {
        return (u / delta); /* all other values */
    }
}
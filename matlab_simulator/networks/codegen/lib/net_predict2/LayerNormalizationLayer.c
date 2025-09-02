/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LayerNormalizationLayer.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "LayerNormalizationLayer.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <xmmintrin.h>

/* Function Definitions */
/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void LayerNormalizationLayer_predict(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.0319612F,  1.04750514F,  1.04845011F, 1.06010664F,
      1.06030202F, 1.03459311F,  1.04801774F, 1.08523655F,
      1.06800747F, 0.998876452F, 1.0345825F,  1.0344826F,
      1.02636325F, 1.08804071F,  1.05733955F, 1.05747283F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void b_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.01794565F,  0.972680211F, 1.07214344F, 1.0406822F,
      0.991535842F, 1.02229655F,  1.04695368F, 1.08015311F,
      1.02145302F,  1.05133677F,  1.01249707F, 1.04022956F,
      1.08889377F,  1.03600585F,  1.06493521F, 1.08624518F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void c_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.05007982F, 1.04579794F, 1.03549039F, 1.06321335F,
      1.0346266F,  1.03457987F, 1.05682385F, 1.06840801F,
      1.00988293F, 1.00069809F, 1.04024541F, 1.0378325F,
      1.03916574F, 1.04779756F, 1.06418014F, 1.0406884F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void d_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.0168891F,   1.00139892F, 0.986827552F, 1.04025972F,
      0.984831214F, 1.02070546F, 1.06379986F,  1.07417035F,
      1.04914057F,  1.04712903F, 1.02403736F,  1.02878153F,
      0.970677257F, 1.04951346F, 1.05652022F,  1.0396924F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void e_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.03406096F, 1.01032031F, 1.0699811F,   1.05958986F,
      1.03400457F, 1.07003105F, 1.03992093F,  1.00676954F,
      1.00769603F, 1.06847954F, 0.998746336F, 1.03713953F,
      1.0195049F,  1.06387091F, 1.03050792F,  1.03269362F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void f_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.01048827F,  1.02427912F, 1.00575006F,  1.03533161F,
      0.997926116F, 1.03973734F, 1.02406F,     1.01328206F,
      1.01909852F,  1.03980494F, 1.03908396F,  1.01273108F,
      1.0509969F,   1.04738557F, 0.972660959F, 1.02429152F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void g_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.05163765F, 1.0330838F,  1.0869174F,  1.06765914F,
      1.03954029F, 1.06079972F, 1.04140508F, 1.06636906F,
      1.00730371F, 1.08393025F, 1.00921416F, 1.06328821F,
      1.05333352F, 1.081967F,   1.06706285F, 1.06781459F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void h_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      0.990923882F, 1.0328083F,  1.01012373F, 1.06852126F,
      1.05565858F,  1.05418074F, 1.05916178F, 1.00624621F,
      1.03954875F,  1.03993642F, 1.06256914F, 1.02129626F,
      1.01179802F,  1.05489516F, 1.05505836F, 1.00794768F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void i_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.02310371F, 1.04507816F, 1.04842401F, 1.05413067F,
      1.04062426F, 1.05981314F, 1.04803669F, 1.01716757F,
      1.05159545F, 1.04621685F, 1.03310454F, 1.02943027F,
      1.05972159F, 1.0937922F,  1.04858088F, 1.04294026F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void j_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.01933742F, 1.00225914F, 1.02596045F, 1.03555882F,
      1.01123071F, 1.02999818F, 1.01985085F, 1.00001192F,
      1.03010213F, 1.02797651F, 1.01669359F, 1.05325782F,
      1.04183924F, 1.04258132F, 1.06673074F, 1.01031411F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void k_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.02086854F, 1.0357343F,  1.04860651F, 1.08375311F,
      1.02141023F, 1.05288517F, 1.03071129F, 1.01339936F,
      1.04905438F, 1.07000148F, 1.03877723F, 1.04128444F,
      1.05111504F, 1.05782485F, 1.01209044F, 1.02581966F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void l_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.0693537F,  1.07172191F, 1.03336895F, 1.06806564F,
      1.02104807F, 1.06684327F, 1.03831732F, 1.01134086F,
      1.0008055F,  1.05087662F, 1.06343782F, 1.02971601F,
      1.05260265F, 1.10238707F, 1.02855945F, 1.02605402F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void m_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.03336227F, 1.08672428F, 1.04434502F, 1.07000673F,
      1.04946899F, 1.03965187F, 1.06744754F, 1.04792964F,
      1.01247847F, 1.06823456F, 1.04286873F, 1.04137826F,
      1.04894149F, 1.07828736F, 1.01947308F, 1.045632F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void n_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.0325886F,  1.04652119F,  1.01991785F, 1.01940548F,
      1.02867794F, 0.969692171F, 1.05333936F, 0.991779923F,
      1.02292931F, 1.03398597F,  1.02649415F, 1.06915236F,
      1.02865124F, 1.01103878F,  1.04063964F, 1.00818837F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void o_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.06803811F, 1.04848266F, 1.10547185F, 1.03934729F,
      1.02726233F, 1.09044182F, 1.08415532F, 1.00455141F,
      1.04328418F, 1.06883979F, 1.04632747F, 1.01595616F,
      1.0508579F,  1.08774233F, 1.01250422F, 1.04900384F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * Arguments    : const float X_Data[160]
 *                float Z_Data[160]
 * Return Type  : void
 */
void p_LayerNormalizationLayer_predi(const float X_Data[160], float Z_Data[160])
{
  static const float fv[16] = {
      1.09254217F, 1.12057793F, 1.10112035F, 1.12580764F,
      1.10919905F, 1.0335952F,  1.04946339F, 1.07552552F,
      1.09103775F, 1.12277222F, 1.11735797F, 1.1173346F,
      1.10117936F, 1.10615098F, 1.07665777F, 1.08797705F};
  __m128 r;
  float b_z_Data[160];
  float zdata[160];
  float z_Data[10];
  float f;
  int k;
  int xi;
  int xpageoffset_tmp;
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = X_Data[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += X_Data[(xpageoffset_tmp + k) + 1];
    }
    f /= 16.0F;
    z_Data[xi] = f;
    r = _mm_set1_ps(f);
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 4],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 8],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r));
    _mm_storeu_ps(&b_z_Data[xpageoffset_tmp + 12],
                  _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r));
  }
  for (k = 0; k < 160; k++) {
    if (rtIsNaNF(b_z_Data[k])) {
      zdata[k] = rtNaNF;
    } else {
      zdata[k] = b_z_Data[k] * b_z_Data[k];
    }
  }
  for (xi = 0; xi < 10; xi++) {
    xpageoffset_tmp = xi << 4;
    f = zdata[xpageoffset_tmp];
    for (k = 0; k < 15; k++) {
      f += zdata[(xpageoffset_tmp + k) + 1];
    }
    __m128 r1;
    r = _mm_set1_ps(z_Data[xi]);
    r1 = _mm_set1_ps(sqrtf(f / 16.0F + 1.0E-5F));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[0]),
            _mm_div_ps(_mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp]), r),
                       r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 4],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[4]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 4]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 8],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[8]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 8]), r),
                r1)));
    _mm_storeu_ps(
        &Z_Data[xpageoffset_tmp + 12],
        _mm_mul_ps(
            _mm_loadu_ps(&fv[12]),
            _mm_div_ps(
                _mm_sub_ps(_mm_loadu_ps(&X_Data[xpageoffset_tmp + 12]), r),
                r1)));
  }
}

/*
 * File trailer for LayerNormalizationLayer.c
 *
 * [EOF]
 */

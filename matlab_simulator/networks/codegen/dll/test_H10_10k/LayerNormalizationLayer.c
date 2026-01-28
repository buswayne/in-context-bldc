/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LayerNormalizationLayer.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Jan-2026 10:52:33
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
      1.03177786F, 1.04785168F, 1.04021919F, 1.05407345F,
      1.0600183F,  1.03189206F, 1.04346371F, 1.07143867F,
      1.06729198F, 1.0009253F,  1.03446186F, 1.02780235F,
      1.02341056F, 1.09062755F, 1.05809653F, 1.05824542F};
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
      1.01331615F,  0.977913618F, 1.04779053F, 1.03778481F,
      0.977139473F, 1.0201124F,   1.03433025F, 1.0662173F,
      1.02063763F,  1.04371822F,  1.00942338F, 1.03998053F,
      1.07200682F,  1.02857387F,  1.06366348F, 1.0744307F};
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
      1.0392158F,  1.04396904F,  1.0325557F,  1.06081796F,
      1.03357565F, 1.03997445F,  1.05488396F, 1.06279504F,
      1.00682294F, 0.994351208F, 1.03922582F, 1.03952301F,
      1.0429498F,  1.04702795F,  1.06564629F, 1.04083848F};
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
      1.00763512F,  0.992605329F, 0.987646163F, 1.02944553F,
      0.978925228F, 1.0054307F,   1.0532527F,   1.04877627F,
      1.03998578F,  1.03604507F,  1.0182066F,   1.02577126F,
      0.97249788F,  1.04869187F,  1.05133474F,  1.03165925F};
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
      1.02386355F, 1.00763965F, 1.07073116F,  1.05384505F,
      1.02781975F, 1.06479073F, 1.0381453F,   1.00119829F,
      1.00672984F, 1.06393731F, 0.999286294F, 1.03404737F,
      1.02869213F, 1.06189191F, 1.02566719F,  1.02887487F};
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
      1.00157166F,  1.01911712F, 1.00545144F,  1.02925503F,
      0.993387461F, 1.03598118F, 1.0216645F,   1.00827801F,
      1.00995886F,  1.03110635F, 1.04262853F,  1.01517904F,
      1.04744959F,  1.03751111F, 0.973311722F, 1.01021814F};
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
      1.03766882F, 1.03307664F, 1.084355F,   1.06273377F,
      1.03869593F, 1.05584335F, 1.04176772F, 1.06417859F,
      1.01112926F, 1.06227052F, 1.00480807F, 1.04971325F,
      1.04306912F, 1.07455635F, 1.06525409F, 1.05904472F};
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
      0.993661582F, 1.02832007F, 1.00836706F, 1.05675125F,
      1.04852033F,  1.05248475F, 1.05647528F, 1.00112152F,
      1.0155077F,   1.03325987F, 1.03921866F, 1.02457702F,
      0.989573121F, 1.04070437F, 1.01862597F, 1.0135715F};
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
      1.02097118F, 1.03929615F, 1.04103422F, 1.04148436F,
      1.03741145F, 1.05417264F, 1.04916573F, 1.0197711F,
      1.04184091F, 1.03991187F, 1.03443515F, 1.03075588F,
      1.04960895F, 1.0751313F,  1.04772329F, 1.04266322F};
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
      1.01428664F, 0.999566674F, 0.9988814F,  1.02868736F,
      1.00080287F, 1.02000463F,  1.01635206F, 0.996965885F,
      1.01659608F, 1.02470088F,  0.995855F,   1.05172181F,
      1.03806615F, 1.03867722F,  1.0531193F,  1.01105094F};
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
      1.01902449F, 1.03329766F, 1.04468942F, 1.06409895F,
      1.02416646F, 1.03335238F, 1.028916F,   1.011446F,
      1.05209494F, 1.06155658F, 1.03621602F, 1.04276133F,
      1.05485916F, 1.04501092F, 1.01352382F, 1.0129019F};
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
      1.06049132F,  1.03938413F, 1.0169028F,  1.05911136F,
      1.01710451F,  1.05585241F, 1.01567495F, 1.00936139F,
      0.996753335F, 1.034621F,   1.04324639F, 1.01639295F,
      1.04946768F,  1.06451571F, 1.02295661F, 1.01714528F};
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
      1.02835488F, 1.07320333F, 1.0262692F,  1.06997824F,
      1.04940104F, 1.03911865F, 1.05383193F, 1.04347634F,
      1.01539898F, 1.05658424F, 1.04184556F, 1.02944386F,
      1.04495192F, 1.06556761F, 1.01129138F, 1.04013562F};
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
      1.02737367F, 1.03308272F,  1.01757824F, 1.0044148F,
      1.02989876F, 0.962158382F, 1.04357672F, 0.998916328F,
      1.02000272F, 1.01187909F,  1.01642716F, 1.05746818F,
      1.03002548F, 1.00777161F,  1.03545272F, 1.00529087F};
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
      1.06198752F, 1.03540826F, 1.04579103F, 1.0292151F,
      1.02538848F, 1.07416928F, 1.0612359F,  1.00286198F,
      1.03155637F, 1.06549668F, 1.04587591F, 1.01146805F,
      1.04162014F, 1.04991949F, 1.01120937F, 1.0276F};
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
      1.08823359F, 1.11597908F, 1.09513056F, 1.1209507F,
      1.10780525F, 1.01957655F, 1.05015159F, 1.0750829F,
      1.09173977F, 1.12202704F, 1.11859727F, 1.11814606F,
      1.09949052F, 1.08777106F, 1.07657027F, 1.0971421F};
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

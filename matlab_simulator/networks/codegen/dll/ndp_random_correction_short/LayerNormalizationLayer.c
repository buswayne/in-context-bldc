/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LayerNormalizationLayer.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:17:18
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
      1.03869545F, 1.05914259F,  1.05429661F, 1.0463841F,
      1.05385554F, 1.03234541F,  1.03927135F, 1.07684457F,
      1.06024349F, 0.993837297F, 1.03051662F, 1.01493239F,
      1.04873586F, 1.10215271F,  1.05833626F, 1.09531951F};
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
      1.02427709F,  0.981380045F, 1.0459609F,  1.04679227F,
      0.997350693F, 1.02653146F,  1.05306399F, 1.06454945F,
      1.03936088F,  1.05228901F,  1.00064969F, 1.06221497F,
      1.04880655F,  1.02589917F,  1.06690061F, 1.09482217F};
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
      1.04757094F,  1.0544883F,   1.03444505F, 1.06120932F,
      1.01937354F,  1.03001916F,  1.07040107F, 1.05360281F,
      1.01254201F,  0.992621243F, 1.0400368F,  1.06340051F,
      0.995890558F, 1.05215013F,  1.0647496F,  1.04192209F};
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
      1.0400089F,   1.02969766F, 0.994024217F, 1.02343345F,
      0.982436061F, 1.04417181F, 1.0448333F,   1.04090142F,
      1.04977322F,  1.06254733F, 1.01688099F,  1.03956902F,
      1.04099286F,  1.05749786F, 1.05606937F,  1.00901067F};
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
      1.03346157F, 1.0145725F,  1.07561505F, 1.08224618F,
      1.03409505F, 1.07071364F, 1.05317533F, 1.00493383F,
      1.00429916F, 1.07351887F, 1.00067639F, 1.04564953F,
      1.05943441F, 1.09045303F, 1.0172261F,  1.02320564F};
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
      1.01115489F, 1.03021324F, 0.988633037F, 1.04937255F,
      1.01384103F, 1.03533816F, 1.03355074F,  1.03710985F,
      1.02500713F, 1.06674063F, 1.05004752F,  1.03425705F,
      1.07733476F, 1.04538512F, 0.986662507F, 1.00851655F};
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
      1.04892993F, 1.02311659F, 1.07855177F, 1.0649699F,
      1.04543734F, 1.06022108F, 1.04739583F, 1.06949973F,
      1.00150645F, 1.06888688F, 1.01621318F, 1.05532944F,
      1.03106976F, 1.06833661F, 1.05982435F, 1.05908179F};
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
      0.990824878F, 1.0312345F,  1.02259958F, 1.06432319F,
      1.06385314F,  1.06374729F, 1.05267298F, 1.01531398F,
      1.01226091F,  1.04111457F, 1.0465517F,  1.03078091F,
      1.03908443F,  1.03295743F, 1.03283238F, 1.01218212F};
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
      1.03886628F, 1.05146945F, 1.037714F,   1.04604793F,
      1.03001249F, 1.05241537F, 1.06052554F, 1.02241027F,
      1.0412668F,  1.05446863F, 1.04111421F, 1.02686942F,
      1.05361772F, 1.07673F,    1.04180992F, 1.05148423F};
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
      0.996035635F, 1.01374006F, 1.01832926F, 1.02618575F,
      1.0261749F,   1.02655292F, 1.03256357F, 1.0134362F,
      1.01854157F,  1.0569489F,  1.00107574F, 1.0675534F,
      1.03503799F,  1.02587605F, 1.05945027F, 1.03284049F};
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
      1.02082527F, 1.00946271F, 1.0560174F,  1.07022464F,
      1.02118194F, 1.04286575F, 1.05336559F, 0.994562864F,
      1.03811681F, 1.06909215F, 1.03643644F, 1.03221512F,
      1.06998861F, 1.05436158F, 1.00581336F, 1.01496327F};
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
      1.08327138F,  1.02826953F, 1.04229534F, 1.06298029F,
      1.02897358F,  1.08218968F, 1.04136884F, 1.00795448F,
      0.987477303F, 1.03875875F, 1.0550983F,  1.05479515F,
      1.03708458F,  1.0711726F,  1.03222847F, 1.01466799F};
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
      1.04439127F, 1.04978538F, 1.04339981F,  1.06967807F,
      1.05393052F, 1.05511475F, 1.05698931F,  1.04253F,
      1.02446294F, 1.04920852F, 1.03621233F,  1.03439343F,
      1.06310844F, 1.0615741F,  0.990895152F, 1.04647374F};
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
      1.03431106F,  1.03061545F,  1.02534473F, 0.994779F,
      1.02778113F,  1.00705862F,  1.04706228F, 1.00705099F,
      1.01926076F,  0.986235201F, 1.01900089F, 1.05965042F,
      0.999771118F, 1.00986F,     1.04532659F, 1.00474191F};
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
      1.05902147F, 1.02225924F, 1.02058089F,  1.01804769F,
      1.02155972F, 1.08376884F, 1.0692544F,   1.02005231F,
      1.02403891F, 1.06240153F, 1.04516506F,  1.0230267F,
      1.04505324F, 1.0451653F,  0.998762786F, 1.02333987F};
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
      1.08126748F, 1.12393343F, 1.08640313F, 1.11003065F,
      1.10465097F, 1.02493799F, 1.05953705F, 1.07393932F,
      1.09658694F, 1.12150133F, 1.12753034F, 1.12219214F,
      1.0315274F,  1.08751667F, 1.07879484F, 1.10210431F};
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

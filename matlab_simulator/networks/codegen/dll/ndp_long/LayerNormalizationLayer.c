/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LayerNormalizationLayer.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:11:19
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
      1.03195107F, 1.0475477F,  1.04639602F, 1.05818713F,
      1.06016302F, 1.03401828F, 1.04612637F, 1.07766771F,
      1.06772685F, 0.99988687F, 1.03448308F, 1.03096306F,
      1.02520454F, 1.08653009F, 1.05784154F, 1.05805635F};
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
      1.01580405F,  0.973516166F, 1.06622601F, 1.0387944F,
      0.983772576F, 1.02102637F,  1.04097652F, 1.07538247F,
      1.02233064F,  1.04923034F,  1.01133823F, 1.03936088F,
      1.07907319F,  1.02879333F,  1.06421101F, 1.08161557F};
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
      1.04598892F, 1.04534853F,  1.03578889F, 1.06229341F,
      1.03429806F, 1.0368973F,   1.05432129F, 1.06694853F,
      1.00998425F, 0.998724937F, 1.04080927F, 1.03673637F,
      1.0404048F,  1.04780853F,  1.06435895F, 1.04036367F};
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
      1.0143342F,   0.997009337F, 0.985594511F, 1.03505921F,
      0.981881857F, 1.0155257F,   1.06342888F,  1.06754076F,
      1.0471915F,   1.04319179F,  1.02192736F,  1.0282284F,
      0.969808042F, 1.05011344F,  1.05521226F,  1.0348115F};
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
      1.03214931F, 1.00951195F, 1.0701741F,   1.05706573F,
      1.03145242F, 1.06971478F, 1.03858018F,  1.00408518F,
      1.00825357F, 1.06556606F, 0.999190807F, 1.0354774F,
      1.02004611F, 1.06299496F, 1.02808845F,  1.03164554F};
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
      1.00692F,     1.023646F,   1.00552201F,  1.03161263F,
      0.995992243F, 1.03796422F, 1.0244751F,   1.01236033F,
      1.01686335F,  1.03721285F, 1.04176784F,  1.01378286F,
      1.04710686F,  1.04532409F, 0.972770751F, 1.02248943F};
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
      1.04660344F, 1.03353822F, 1.08483386F, 1.06715429F,
      1.03857219F, 1.05928993F, 1.04145157F, 1.06648612F,
      1.01293981F, 1.07894039F, 1.00696206F, 1.05698502F,
      1.05184257F, 1.08001029F, 1.06522012F, 1.06382942F};
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
      0.993647277F, 1.03011584F, 1.01149452F, 1.06565678F,
      1.05202651F,  1.05315578F, 1.05628026F, 1.00418413F,
      1.03705287F,  1.03883433F, 1.05519652F, 1.0211364F,
      1.0042603F,   1.05078423F, 1.05192041F, 1.01258743F};
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
      1.02215445F, 1.04407609F, 1.04689825F, 1.04834032F,
      1.03969955F, 1.05674732F, 1.04921746F, 1.01786315F,
      1.04687107F, 1.04394865F, 1.03265774F, 1.02868474F,
      1.05537105F, 1.08909678F, 1.04739094F, 1.04500175F};
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
      1.01794732F, 1.00138199F, 1.02397823F, 1.03065205F,
      1.00577796F, 1.02439225F, 1.01815F,    0.998566389F,
      1.02332735F, 1.02807117F, 1.01221764F, 1.05292666F,
      1.0401926F,  1.04153728F, 1.06329215F, 1.01063204F};
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
      1.02136552F, 1.03437054F, 1.04695451F, 1.07911801F,
      1.02065337F, 1.04839885F, 1.03106058F, 1.01200044F,
      1.050349F,   1.06704009F, 1.0353781F,  1.03989792F,
      1.05257416F, 1.05275524F, 1.01354718F, 1.02116191F};
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
      1.06852388F,  1.06072378F, 1.03162086F, 1.06481838F,
      1.01938534F,  1.06766188F, 1.02551818F, 1.01098645F,
      0.998485863F, 1.04871535F, 1.05145442F, 1.02478051F,
      1.05100596F,  1.09493053F, 1.0241636F,  1.02300382F};
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
      1.03363752F, 1.08605015F, 1.03772199F, 1.07058859F,
      1.0505172F,  1.03942F,    1.0640415F,  1.04581809F,
      1.01315808F, 1.06116915F, 1.04068911F, 1.03919411F,
      1.04767966F, 1.07291842F, 1.01736283F, 1.04419267F};
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
      1.0331018F,  1.04174471F,  1.01887774F, 1.01629937F,
      1.02901495F, 0.968612194F, 1.05311549F, 0.994670093F,
      1.02227342F, 1.02773082F,  1.02445626F, 1.06671023F,
      1.03111577F, 1.01056552F,  1.03878331F, 1.00651634F};
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
      1.06580126F, 1.04450178F, 1.08080184F, 1.03748167F,
      1.02753305F, 1.08440053F, 1.06712019F, 1.00435197F,
      1.03693485F, 1.06753266F, 1.04628563F, 1.01467299F,
      1.04548597F, 1.0784266F,  1.01261175F, 1.03560662F};
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
      1.09232318F, 1.12142742F, 1.10026014F, 1.12719119F,
      1.10934305F, 1.02771318F, 1.05051744F, 1.07596231F,
      1.09182453F, 1.12353349F, 1.11811209F, 1.11770725F,
      1.09870243F, 1.10065043F, 1.07718563F, 1.0893892F};
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

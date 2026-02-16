/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: attentionForward.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:12:46
 */

/* Include Files */
#include "attentionForward.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>
#include <xmmintrin.h>

/* Function Definitions */
/*
 * Arguments    : const float Q[320]
 *                const float K[320]
 *                const float V[320]
 *                float X[320]
 * Return Type  : void
 */
void attentionForward(const float Q[320], const float K[320],
                      const float V[320], float X[320])
{
  static const float fv[100] = {
      0.0F,     1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F,
      1.0E+10F, 1.0E+10F, 1.0E+10F, 0.0F,     0.0F,     1.0E+10F, 1.0E+10F,
      1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 0.0F,
      0.0F,     0.0F,     1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F,
      1.0E+10F, 1.0E+10F, 0.0F,     0.0F,     0.0F,     0.0F,     1.0E+10F,
      1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 0.0F,     0.0F,
      0.0F,     0.0F,     0.0F,     1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F,
      1.0E+10F, 0.0F,     0.0F,     0.0F,     0.0F,     0.0F,     0.0F,
      1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 0.0F,     0.0F,     0.0F,
      0.0F,     0.0F,     0.0F,     0.0F,     1.0E+10F, 1.0E+10F, 1.0E+10F,
      0.0F,     0.0F,     0.0F,     0.0F,     0.0F,     0.0F,     0.0F,
      0.0F,     1.0E+10F, 1.0E+10F, 0.0F,     0.0F,     0.0F,     0.0F,
      0.0F,     0.0F,     0.0F,     0.0F,     0.0F,     1.0E+10F, 0.0F,
      0.0F,     0.0F,     0.0F,     0.0F,     0.0F,     0.0F,     0.0F,
      0.0F,     0.0F};
  static const signed char iv[100] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  __m128 r;
  float b_K[320];
  float b_Q[320];
  float b_V[320];
  float W[200];
  float X_Data[200];
  float dataOut_Data[200];
  float dataExp[10];
  float s;
  int b_i;
  int boffset;
  int coffset;
  int i;
  int i1;
  int idx;
  int j;
  int k;
  for (k = 0; k < 10; k++) {
    for (idx = 0; idx < 2; idx++) {
      memcpy(&b_Q[k * 16 + idx * 160], &Q[k * 32 + idx * 16],
             16U * sizeof(float));
      memcpy(&b_K[k * 16 + idx * 160], &K[k * 32 + idx * 16],
             16U * sizeof(float));
      memcpy(&b_V[k * 16 + idx * 160], &V[k * 32 + idx * 16],
             16U * sizeof(float));
    }
  }
  for (idx = 0; idx < 2; idx++) {
    for (j = 0; j < 10; j++) {
      coffset = j * 10;
      boffset = j << 4;
      for (b_i = 0; b_i < 10; b_i++) {
        int aoffset;
        aoffset = b_i << 4;
        s = 0.0F;
        for (k = 0; k < 16; k++) {
          i = aoffset + k;
          i1 = boffset + k;
          s += b_K[(i % 16 + ((i / 16) << 4)) + 160 * idx] *
               b_Q[(i1 % 16 + ((i1 / 16) << 4)) + 160 * idx];
        }
        i = coffset + b_i;
        W[(i % 10 + 10 * (i / 10)) + 100 * idx] = s;
      }
    }
  }
  for (i = 0; i <= 196; i += 4) {
    r = _mm_loadu_ps(&W[i]);
    _mm_storeu_ps(&W[i], _mm_mul_ps(r, _mm_set1_ps(0.25F)));
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 100; i1++) {
      idx = i1 + i * 100;
      s = W[idx] * (float)iv[i1] - fv[i1];
      dataOut_Data[idx] = s;
      X_Data[idx] = s;
    }
  }
  for (j = 0; j < 20; j++) {
    float maxVal;
    coffset = j / 10;
    boffset = j - coffset * 10;
    i = 10 * boffset + 100 * coffset;
    if (!rtIsNaNF(dataOut_Data[i])) {
      idx = 1;
    } else {
      boolean_T exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 11)) {
        if (!rtIsNaNF(dataOut_Data[((k + 10 * boffset) + 100 * coffset) - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      maxVal = X_Data[i];
    } else {
      maxVal = X_Data[((idx + 10 * boffset) + 100 * coffset) - 1];
      i1 = idx + 1;
      for (k = i1; k < 11; k++) {
        s = X_Data[((k + 10 * boffset) + 100 * coffset) - 1];
        if (maxVal < s) {
          maxVal = s;
        }
      }
    }
    for (idx = 0; idx < 10; idx++) {
      dataExp[idx] =
          expf(dataOut_Data[(idx + 10 * boffset) + 100 * coffset] - maxVal);
    }
    s = dataExp[0];
    for (k = 0; k < 9; k++) {
      s += dataExp[k + 1];
    }
    __m128 r1;
    r = _mm_loadu_ps(&dataExp[0]);
    r1 = _mm_set1_ps(s);
    _mm_storeu_ps(&W[i], _mm_div_ps(r, r1));
    r = _mm_loadu_ps(&dataExp[4]);
    _mm_storeu_ps(&W[i + 4], _mm_div_ps(r, r1));
    W[i + 8] = dataExp[8] / s;
    W[i + 9] = dataExp[9] / s;
  }
  for (idx = 0; idx < 2; idx++) {
    for (j = 0; j < 10; j++) {
      coffset = j << 4;
      boffset = j * 10;
      for (b_i = 0; b_i < 16; b_i++) {
        s = 0.0F;
        for (k = 0; k < 10; k++) {
          i = (k << 4) + b_i;
          i1 = boffset + k;
          s += b_V[(i % 16 + ((i / 16) << 4)) + 160 * idx] *
               W[(i1 % 10 + 10 * (i1 / 10)) + 100 * idx];
        }
        i = coffset + b_i;
        b_Q[(i % 16 + ((i / 16) << 4)) + 160 * idx] = s;
      }
    }
  }
  for (k = 0; k < 2; k++) {
    for (idx = 0; idx < 10; idx++) {
      memcpy(&X[k * 16 + idx * 32], &b_Q[k * 160 + idx * 16],
             16U * sizeof(float));
    }
  }
}

/*
 * File trailer for attentionForward.c
 *
 * [EOF]
 */

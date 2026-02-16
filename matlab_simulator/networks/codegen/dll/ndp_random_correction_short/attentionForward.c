/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: attentionForward.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 16-Feb-2026 17:17:18
 */

/* Include Files */
#include "attentionForward.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <xmmintrin.h>

/* Function Definitions */
/*
 * Arguments    : const float Q[160]
 *                const float K[160]
 *                const float V[160]
 *                float X[160]
 * Return Type  : void
 */
void attentionForward(const float Q[160], const float K[160],
                      const float V[160], float X[160])
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
  float W[400];
  float X_Data[400];
  float dataOut_Data[400];
  float b_K[160];
  float b_Q[160];
  float b_V[160];
  float dataExp[10];
  float sumX;
  int aoffset;
  int b_i;
  int b_k;
  int boffset;
  int coffset;
  int i;
  int i1;
  int idx;
  int k;
  for (k = 0; k < 10; k++) {
    for (b_k = 0; b_k < 4; b_k++) {
      idx = (b_k << 2) + (k << 4);
      aoffset = (k << 2) + 40 * b_k;
      b_Q[aoffset] = Q[idx];
      b_Q[aoffset + 1] = Q[idx + 1];
      b_Q[aoffset + 2] = Q[idx + 2];
      b_Q[aoffset + 3] = Q[idx + 3];
      b_K[aoffset] = K[idx];
      b_K[aoffset + 1] = K[idx + 1];
      b_K[aoffset + 2] = K[idx + 2];
      b_K[aoffset + 3] = K[idx + 3];
      b_V[aoffset] = V[idx];
      b_V[aoffset + 1] = V[idx + 1];
      b_V[aoffset + 2] = V[idx + 2];
      b_V[aoffset + 3] = V[idx + 3];
    }
  }
  for (b_k = 0; b_k < 4; b_k++) {
    for (idx = 0; idx < 10; idx++) {
      coffset = idx * 10;
      boffset = idx << 2;
      for (b_i = 0; b_i < 10; b_i++) {
        aoffset = b_i << 2;
        i = coffset + b_i;
        W[(i % 10 + 10 * (i / 10)) + 100 * b_k] =
            ((b_K[(aoffset % 4 + ((aoffset / 4) << 2)) + 40 * b_k] *
                  b_Q[(boffset % 4 + ((boffset / 4) << 2)) + 40 * b_k] +
              b_K[((aoffset + 1) % 4 + (((aoffset + 1) / 4) << 2)) + 40 * b_k] *
                  b_Q[((boffset + 1) % 4 + (((boffset + 1) / 4) << 2)) +
                      40 * b_k]) +
             b_K[((aoffset + 2) % 4 + (((aoffset + 2) / 4) << 2)) + 40 * b_k] *
                 b_Q[((boffset + 2) % 4 + (((boffset + 2) / 4) << 2)) +
                     40 * b_k]) +
            b_K[((aoffset + 3) % 4 + (((aoffset + 3) / 4) << 2)) + 40 * b_k] *
                b_Q[((boffset + 3) % 4 + (((boffset + 3) / 4) << 2)) +
                    40 * b_k];
      }
    }
  }
  for (i = 0; i <= 396; i += 4) {
    r = _mm_loadu_ps(&W[i]);
    _mm_storeu_ps(&W[i], _mm_mul_ps(r, _mm_set1_ps(0.5F)));
  }
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 100; i1++) {
      idx = i1 + i * 100;
      sumX = W[idx] * (float)iv[i1] - fv[i1];
      dataOut_Data[idx] = sumX;
      X_Data[idx] = sumX;
    }
  }
  for (b_k = 0; b_k < 40; b_k++) {
    float maxVal;
    aoffset = b_k / 10;
    coffset = b_k - aoffset * 10;
    i = 10 * coffset + 100 * aoffset;
    if (!rtIsNaNF(dataOut_Data[i])) {
      idx = 1;
    } else {
      boolean_T exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 11)) {
        if (!rtIsNaNF(dataOut_Data[((k + 10 * coffset) + 100 * aoffset) - 1])) {
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
      maxVal = X_Data[((idx + 10 * coffset) + 100 * aoffset) - 1];
      i1 = idx + 1;
      for (k = i1; k < 11; k++) {
        sumX = X_Data[((k + 10 * coffset) + 100 * aoffset) - 1];
        if (maxVal < sumX) {
          maxVal = sumX;
        }
      }
    }
    for (idx = 0; idx < 10; idx++) {
      dataExp[idx] =
          expf(dataOut_Data[(idx + 10 * coffset) + 100 * aoffset] - maxVal);
    }
    sumX = dataExp[0];
    for (k = 0; k < 9; k++) {
      sumX += dataExp[k + 1];
    }
    __m128 r1;
    r = _mm_loadu_ps(&dataExp[0]);
    r1 = _mm_set1_ps(sumX);
    _mm_storeu_ps(&W[i], _mm_div_ps(r, r1));
    r = _mm_loadu_ps(&dataExp[4]);
    _mm_storeu_ps(&W[i + 4], _mm_div_ps(r, r1));
    W[i + 8] = dataExp[8] / sumX;
    W[i + 9] = dataExp[9] / sumX;
  }
  for (b_k = 0; b_k < 4; b_k++) {
    for (idx = 0; idx < 10; idx++) {
      coffset = idx << 2;
      boffset = idx * 10;
      for (b_i = 0; b_i < 4; b_i++) {
        sumX = 0.0F;
        for (k = 0; k < 10; k++) {
          i = (k << 2) + b_i;
          i1 = boffset + k;
          sumX += b_V[(i % 4 + ((i / 4) << 2)) + 40 * b_k] *
                  W[(i1 % 10 + 10 * (i1 / 10)) + 100 * b_k];
        }
        i = coffset + b_i;
        b_Q[(i % 4 + ((i / 4) << 2)) + 40 * b_k] = sumX;
      }
    }
  }
  for (k = 0; k < 4; k++) {
    for (b_k = 0; b_k < 10; b_k++) {
      idx = (b_k << 2) + 40 * k;
      aoffset = (k << 2) + (b_k << 4);
      X[aoffset] = b_Q[idx];
      X[aoffset + 1] = b_Q[idx + 1];
      X[aoffset + 2] = b_Q[idx + 2];
      X[aoffset + 3] = b_Q[idx + 3];
    }
  }
}

/*
 * File trailer for attentionForward.c
 *
 * [EOF]
 */

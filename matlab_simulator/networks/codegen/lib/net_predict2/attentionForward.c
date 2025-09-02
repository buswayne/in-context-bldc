/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: attentionForward.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 01-Sep-2025 14:59:08
 */

/* Include Files */
#include "attentionForward.h"
#include "internal_softmax.h"
#include "rt_nonfinite.h"
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
  float W[400];
  float X_Data[400];
  float b_K[160];
  float b_Q[160];
  float b_V[160];
  int Q_tmp_tmp;
  int aoffset;
  int b_i;
  int b_k;
  int boffset;
  int coffset;
  int i;
  int i1;
  int k;
  for (k = 0; k < 10; k++) {
    for (b_k = 0; b_k < 4; b_k++) {
      aoffset = (b_k << 2) + (k << 4);
      Q_tmp_tmp = (k << 2) + 40 * b_k;
      b_Q[Q_tmp_tmp] = Q[aoffset];
      b_Q[Q_tmp_tmp + 1] = Q[aoffset + 1];
      b_Q[Q_tmp_tmp + 2] = Q[aoffset + 2];
      b_Q[Q_tmp_tmp + 3] = Q[aoffset + 3];
      b_K[Q_tmp_tmp] = K[aoffset];
      b_K[Q_tmp_tmp + 1] = K[aoffset + 1];
      b_K[Q_tmp_tmp + 2] = K[aoffset + 2];
      b_K[Q_tmp_tmp + 3] = K[aoffset + 3];
      b_V[Q_tmp_tmp] = V[aoffset];
      b_V[Q_tmp_tmp + 1] = V[aoffset + 1];
      b_V[Q_tmp_tmp + 2] = V[aoffset + 2];
      b_V[Q_tmp_tmp + 3] = V[aoffset + 3];
    }
  }
  for (Q_tmp_tmp = 0; Q_tmp_tmp < 4; Q_tmp_tmp++) {
    for (b_k = 0; b_k < 10; b_k++) {
      coffset = b_k * 10;
      boffset = b_k << 2;
      for (b_i = 0; b_i < 10; b_i++) {
        aoffset = b_i << 2;
        i = coffset + b_i;
        W[(i % 10 + 10 * (i / 10)) + 100 * Q_tmp_tmp] =
            ((b_K[(aoffset % 4 + ((aoffset / 4) << 2)) + 40 * Q_tmp_tmp] *
                  b_Q[(boffset % 4 + ((boffset / 4) << 2)) + 40 * Q_tmp_tmp] +
              b_K[((aoffset + 1) % 4 + (((aoffset + 1) / 4) << 2)) +
                  40 * Q_tmp_tmp] *
                  b_Q[((boffset + 1) % 4 + (((boffset + 1) / 4) << 2)) +
                      40 * Q_tmp_tmp]) +
             b_K[((aoffset + 2) % 4 + (((aoffset + 2) / 4) << 2)) +
                 40 * Q_tmp_tmp] *
                 b_Q[((boffset + 2) % 4 + (((boffset + 2) / 4) << 2)) +
                     40 * Q_tmp_tmp]) +
            b_K[((aoffset + 3) % 4 + (((aoffset + 3) / 4) << 2)) +
                40 * Q_tmp_tmp] *
                b_Q[((boffset + 3) % 4 + (((boffset + 3) / 4) << 2)) +
                    40 * Q_tmp_tmp];
      }
    }
  }
  for (i = 0; i <= 396; i += 4) {
    __m128 r;
    r = _mm_loadu_ps(&W[i]);
    _mm_storeu_ps(&W[i], _mm_mul_ps(r, _mm_set1_ps(0.5F)));
  }
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 100; i1++) {
      aoffset = i1 + i * 100;
      W[aoffset] = W[aoffset] * (float)iv[i1] - fv[i1];
    }
  }
  iComputeSoftmaxForCpu(W, X_Data);
  for (Q_tmp_tmp = 0; Q_tmp_tmp < 4; Q_tmp_tmp++) {
    for (b_k = 0; b_k < 10; b_k++) {
      coffset = b_k << 2;
      boffset = b_k * 10;
      for (b_i = 0; b_i < 4; b_i++) {
        float s;
        s = 0.0F;
        for (k = 0; k < 10; k++) {
          i = (k << 2) + b_i;
          i1 = boffset + k;
          s += b_V[(i % 4 + ((i / 4) << 2)) + 40 * Q_tmp_tmp] *
               X_Data[(i1 % 10 + 10 * (i1 / 10)) + 100 * Q_tmp_tmp];
        }
        i = coffset + b_i;
        b_Q[(i % 4 + ((i / 4) << 2)) + 40 * Q_tmp_tmp] = s;
      }
    }
  }
  for (k = 0; k < 4; k++) {
    for (b_k = 0; b_k < 10; b_k++) {
      aoffset = (b_k << 2) + 40 * k;
      Q_tmp_tmp = (k << 2) + (b_k << 4);
      X[Q_tmp_tmp] = b_Q[aoffset];
      X[Q_tmp_tmp + 1] = b_Q[aoffset + 1];
      X[Q_tmp_tmp + 2] = b_Q[aoffset + 2];
      X[Q_tmp_tmp + 3] = b_Q[aoffset + 3];
    }
  }
}

/*
 * File trailer for attentionForward.c
 *
 * [EOF]
 */

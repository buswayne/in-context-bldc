/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeSelfAttention.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Jan-2026 10:49:46
 */

/* Include Files */
#include "computeSelfAttention.h"
#include "rt_nonfinite.h"
#include <xmmintrin.h>

/* Function Definitions */
/*
 * Arguments    : const float X[160]
 *                const float QW[256]
 *                const float KW[256]
 *                const float VW[256]
 *                const float QB[16]
 *                const float KB[16]
 *                const float VB[16]
 *                float Q[160]
 *                float K[160]
 *                float V[160]
 * Return Type  : void
 */
void iInputLinearProjection(const float X[160], const float QW[256],
                            const float KW[256], const float VW[256],
                            const float QB[16], const float KB[16],
                            const float VB[16], float Q[160], float K[160],
                            float V[160])
{
  int coffset_tmp_tmp;
  int i;
  int j;
  int k;
  for (j = 0; j < 10; j++) {
    coffset_tmp_tmp = j << 4;
    for (i = 0; i < 16; i++) {
      float b_s;
      float c_s;
      float s;
      int K_tmp;
      s = 0.0F;
      b_s = 0.0F;
      K_tmp = i + coffset_tmp_tmp;
      c_s = 0.0F;
      for (k = 0; k < 16; k++) {
        float f;
        int s_tmp;
        f = X[coffset_tmp_tmp + k];
        s_tmp = (k << 4) + i;
        s += QW[s_tmp] * f;
        b_s += KW[s_tmp] * f;
        c_s += VW[s_tmp] * f;
      }
      Q[i + coffset_tmp_tmp] = s + QB[i];
      K[K_tmp] = b_s + KB[i];
      V[K_tmp] = c_s;
    }
  }
  for (j = 0; j < 10; j++) {
    __m128 r;
    coffset_tmp_tmp = j << 4;
    r = _mm_loadu_ps(&V[coffset_tmp_tmp]);
    _mm_storeu_ps(&V[coffset_tmp_tmp], _mm_add_ps(r, _mm_loadu_ps(&VB[0])));
    r = _mm_loadu_ps(&V[coffset_tmp_tmp + 4]);
    _mm_storeu_ps(&V[coffset_tmp_tmp + 4], _mm_add_ps(r, _mm_loadu_ps(&VB[4])));
    r = _mm_loadu_ps(&V[coffset_tmp_tmp + 8]);
    _mm_storeu_ps(&V[coffset_tmp_tmp + 8], _mm_add_ps(r, _mm_loadu_ps(&VB[8])));
    r = _mm_loadu_ps(&V[coffset_tmp_tmp + 12]);
    _mm_storeu_ps(&V[coffset_tmp_tmp + 12],
                  _mm_add_ps(r, _mm_loadu_ps(&VB[12])));
  }
}

/*
 * File trailer for computeSelfAttention.c
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * attentionForward.c
 *
 * Code generation for function 'attentionForward'
 *
 */

/* Include files */
#include "attentionForward.h"
#include "internal_softmax.h"
#include "rt_nonfinite.h"
#include <xmmintrin.h>

/* Variable Definitions */
static emlrtRSInfo kb_emlrtRSI = {
    44,                 /* lineNo */
    "attentionForward", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\nnet\\cnn\\+nnet\\+internal\\+cnn\\+"
    "coder\\+layer\\+utils\\+attentionUtils\\attentio"
    "nForward.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    42,                /* lineNo */
    "dlarray/softmax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\nnet\\deepcoder\\+deep\\+internal\\+"
    "coder\\@dlarray\\softmax.m" /* pathName */
};

static emlrtRSInfo mb_emlrtRSI = {
    66,                 /* lineNo */
    "internal_softmax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\nnet\\deepcoder\\+deep\\+internal\\+"
    "coder\\+dlarray\\internal_softmax.m" /* pathName */
};

/* Function Definitions */
/*
 *
 */
void attentionForward(const emlrtStack *sp, const real32_T Q[160],
                      const real32_T K[160], const real32_T V[160],
                      real32_T X[160])
{
  static const real32_T fv[100] = {
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
  static const int8_T iv[100] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T aoffset;
  int32_T b_i;
  int32_T boffset;
  int32_T coffset;
  int32_T i;
  int32_T i1;
  int32_T ii;
  int32_T j;
  int32_T k;
  real32_T W[400];
  real32_T X_Data[400];
  real32_T b_K[160];
  real32_T b_Q[160];
  real32_T b_V[160];
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  for (k = 0; k < 10; k++) {
    for (boffset = 0; boffset < 4; boffset++) {
      aoffset = (boffset << 2) + (k << 4);
      coffset = (k << 2) + 40 * boffset;
      b_Q[coffset] = Q[aoffset];
      b_Q[coffset + 1] = Q[aoffset + 1];
      b_Q[coffset + 2] = Q[aoffset + 2];
      b_Q[coffset + 3] = Q[aoffset + 3];
      b_K[coffset] = K[aoffset];
      b_K[coffset + 1] = K[aoffset + 1];
      b_K[coffset + 2] = K[aoffset + 2];
      b_K[coffset + 3] = K[aoffset + 3];
      b_V[coffset] = V[aoffset];
      b_V[coffset + 1] = V[aoffset + 1];
      b_V[coffset + 2] = V[aoffset + 2];
      b_V[coffset + 3] = V[aoffset + 3];
    }
  }
  for (ii = 0; ii < 4; ii++) {
    for (j = 0; j < 10; j++) {
      coffset = j * 10;
      boffset = j << 2;
      for (b_i = 0; b_i < 10; b_i++) {
        aoffset = b_i << 2;
        i = coffset + b_i;
        W[(i % 10 + 10 * (i / 10)) + 100 * ii] =
            ((b_K[(aoffset % 4 + ((aoffset / 4) << 2)) + 40 * ii] *
                  b_Q[(boffset % 4 + ((boffset / 4) << 2)) + 40 * ii] +
              b_K[((aoffset + 1) % 4 + (((aoffset + 1) / 4) << 2)) + 40 * ii] *
                  b_Q[((boffset + 1) % 4 + (((boffset + 1) / 4) << 2)) +
                      40 * ii]) +
             b_K[((aoffset + 2) % 4 + (((aoffset + 2) / 4) << 2)) + 40 * ii] *
                 b_Q[((boffset + 2) % 4 + (((boffset + 2) / 4) << 2)) +
                     40 * ii]) +
            b_K[((aoffset + 3) % 4 + (((aoffset + 3) / 4) << 2)) + 40 * ii] *
                b_Q[((boffset + 3) % 4 + (((boffset + 3) / 4) << 2)) + 40 * ii];
      }
    }
  }
  for (i = 0; i <= 396; i += 4) {
    __m128 r;
    r = _mm_loadu_ps(&W[i]);
    _mm_storeu_ps(&W[i], _mm_mul_ps(r, _mm_set1_ps(0.5F)));
  }
  st.site = &kb_emlrtRSI;
  b_st.site = &lb_emlrtRSI;
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 100; i1++) {
      aoffset = i1 + i * 100;
      W[aoffset] = W[aoffset] * (real32_T)iv[i1] - fv[i1];
    }
  }
  c_st.site = &mb_emlrtRSI;
  iComputeSoftmaxForCpu(&c_st, W, X_Data);
  for (ii = 0; ii < 4; ii++) {
    for (j = 0; j < 10; j++) {
      coffset = j << 2;
      boffset = j * 10;
      for (b_i = 0; b_i < 4; b_i++) {
        real32_T s;
        s = 0.0F;
        for (k = 0; k < 10; k++) {
          i = (k << 2) + b_i;
          i1 = boffset + k;
          s += b_V[(i % 4 + ((i / 4) << 2)) + 40 * ii] *
               X_Data[(i1 % 10 + 10 * (i1 / 10)) + 100 * ii];
        }
        i = coffset + b_i;
        b_Q[(i % 4 + ((i / 4) << 2)) + 40 * ii] = s;
      }
    }
  }
  for (k = 0; k < 4; k++) {
    for (boffset = 0; boffset < 10; boffset++) {
      aoffset = (boffset << 2) + 40 * k;
      coffset = (k << 2) + (boffset << 4);
      X[coffset] = b_Q[aoffset];
      X[coffset + 1] = b_Q[aoffset + 1];
      X[coffset + 2] = b_Q[aoffset + 2];
      X[coffset + 3] = b_Q[aoffset + 3];
    }
  }
}

/* End of code generation (attentionForward.c) */

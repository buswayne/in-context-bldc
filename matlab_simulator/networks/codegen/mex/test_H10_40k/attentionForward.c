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
#include "elementwiseOperationInPlace.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "test_H10_40k_data.h"
#include "mwmathutil.h"
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

static emlrtBCInfo c_emlrtBCI = {
    1,                       /* iFirst */
    10,                      /* iLast */
    116,                     /* lineNo */
    21,                      /* colNo */
    "",                      /* aName */
    "iComputeSoftmaxForCpu", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\nnet\\deepcoder\\+deep\\+internal\\+"
    "coder\\+dlarray\\internal_softmax.m", /* pName */
    0                                      /* checkKind */
};

/* Function Definitions */
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
  __m128 r;
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
  real32_T dataOut_Data[400];
  real32_T b_K[160];
  real32_T b_Q[160];
  real32_T b_V[160];
  real32_T dataExp[10];
  real32_T s;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  for (k = 0; k < 10; k++) {
    for (boffset = 0; boffset < 4; boffset++) {
      coffset = (boffset << 2) + (k << 4);
      aoffset = (k << 2) + 40 * boffset;
      b_Q[aoffset] = Q[coffset];
      b_Q[aoffset + 1] = Q[coffset + 1];
      b_Q[aoffset + 2] = Q[coffset + 2];
      b_Q[aoffset + 3] = Q[coffset + 3];
      b_K[aoffset] = K[coffset];
      b_K[aoffset + 1] = K[coffset + 1];
      b_K[aoffset + 2] = K[coffset + 2];
      b_K[aoffset + 3] = K[coffset + 3];
      b_V[aoffset] = V[coffset];
      b_V[aoffset + 1] = V[coffset + 1];
      b_V[aoffset + 2] = V[coffset + 2];
      b_V[aoffset + 3] = V[coffset + 3];
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
    r = _mm_loadu_ps(&W[i]);
    _mm_storeu_ps(&W[i], _mm_mul_ps(r, _mm_set1_ps(0.5F)));
  }
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 100; i1++) {
      coffset = i1 + i * 100;
      dataOut_Data[coffset] = W[coffset] * (real32_T)iv[i1] - fv[i1];
    }
  }
  st.site = &kb_emlrtRSI;
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 100; i1++) {
      coffset = i1 + i * 100;
      X_Data[coffset] = W[coffset] * (real32_T)iv[i1] - fv[i1];
    }
  }
  b_st.site = &lb_emlrtRSI;
  c_st.site = &mb_emlrtRSI;
  for (ii = 0; ii < 40; ii++) {
    __m128 r1;
    real32_T maxVal;
    aoffset = ii / 10;
    boffset = ii - aoffset * 10;
    if ((boffset + 1 < 1) || (boffset + 1 > 10)) {
      emlrtDynamicBoundsCheckR2012b(boffset + 1, 1, 10, &c_emlrtBCI, &c_st);
    }
    i = 10 * boffset + 100 * aoffset;
    if (!muSingleScalarIsNaN(X_Data[i])) {
      coffset = 1;
    } else {
      boolean_T exitg1;
      coffset = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 11)) {
        if (!muSingleScalarIsNaN(
                X_Data[((k + 10 * boffset) + 100 * aoffset) - 1])) {
          coffset = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (coffset == 0) {
      maxVal = dataOut_Data[i];
    } else {
      maxVal = dataOut_Data[((coffset + 10 * boffset) + 100 * aoffset) - 1];
      i1 = coffset + 1;
      for (k = i1; k < 11; k++) {
        s = dataOut_Data[((k + 10 * boffset) + 100 * aoffset) - 1];
        if (maxVal < s) {
          maxVal = s;
        }
      }
    }
    r = _mm_loadu_ps(&X_Data[i]);
    r1 = _mm_set1_ps(maxVal);
    _mm_storeu_ps(&dataExp[0], _mm_sub_ps(r, r1));
    r = _mm_loadu_ps(&X_Data[i + 4]);
    _mm_storeu_ps(&dataExp[4], _mm_sub_ps(r, r1));
    dataExp[8] = X_Data[i + 8] - maxVal;
    dataExp[9] = X_Data[i + 9] - maxVal;
    lambdaForColumnMajorGeneric(dataExp);
    s = b_sumColumnB(dataExp);
    r = _mm_loadu_ps(&dataExp[0]);
    r1 = _mm_set1_ps(s);
    _mm_storeu_ps(&W[i], _mm_div_ps(r, r1));
    r = _mm_loadu_ps(&dataExp[4]);
    _mm_storeu_ps(&W[i + 4], _mm_div_ps(r, r1));
    W[i + 8] = dataExp[8] / s;
    W[i + 9] = dataExp[9] / s;
  }
  for (ii = 0; ii < 4; ii++) {
    for (j = 0; j < 10; j++) {
      coffset = j << 2;
      boffset = j * 10;
      for (b_i = 0; b_i < 4; b_i++) {
        s = 0.0F;
        for (k = 0; k < 10; k++) {
          i = (k << 2) + b_i;
          i1 = boffset + k;
          s += b_V[(i % 4 + ((i / 4) << 2)) + 40 * ii] *
               W[(i1 % 10 + 10 * (i1 / 10)) + 100 * ii];
        }
        i = coffset + b_i;
        b_Q[(i % 4 + ((i / 4) << 2)) + 40 * ii] = s;
      }
    }
  }
  for (k = 0; k < 4; k++) {
    for (boffset = 0; boffset < 10; boffset++) {
      coffset = (boffset << 2) + 40 * k;
      aoffset = (k << 2) + (boffset << 4);
      X[aoffset] = b_Q[coffset];
      X[aoffset + 1] = b_Q[coffset + 1];
      X[aoffset + 2] = b_Q[coffset + 2];
      X[aoffset + 3] = b_Q[coffset + 3];
    }
  }
}

/* End of code generation (attentionForward.c) */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * internal_softmax.c
 *
 * Code generation for function 'internal_softmax'
 *
 */

/* Include files */
#include "internal_softmax.h"
#include "elementwiseOperationInPlace.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "mwmathutil.h"
#include "omp.h"
#include <xmmintrin.h>

/* Variable Definitions */
static emlrtBCInfo emlrtBCI = {
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

static emlrtBCInfo b_emlrtBCI = {
    1,                       /* iFirst */
    10,                      /* iLast */
    134,                     /* lineNo */
    21,                      /* colNo */
    "",                      /* aName */
    "iComputeSoftmaxForCpu", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\nnet\\deepcoder\\+deep\\+internal\\+"
    "coder\\+dlarray\\internal_softmax.m", /* pName */
    3                                      /* checkKind */
};

static emlrtRSInfo xb_emlrtRSI = {
    1,                             /* lineNo */
    "elementwiseOperationInPlace", /* fcnName */
    "C:"
    "\\ProgramData\\MATLAB\\SupportPackages\\R2024b\\toolbox\\shared\\dlcoder_"
    "base\\supportpackages\\shared_dl_targets\\+coder\\+in"
    "ternal\\+layer\\elementwiseOperationInPlace.p" /* pathName */
};

/* Function Definitions */
/*
 *
 */
void iComputeSoftmaxForCpu(const emlrtStack *sp, const real32_T xdata[400],
                           real32_T ydata[400])
{
  __m128 r;
  __m128 r1;
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack b_st;
  emlrtStack st;
  real_T nonChannelSubscriptIndices[3];
  int32_T i;
  int32_T iComputeSoftmaxForCpu_numThreads;
  int32_T idx;
  int32_T k;
  int32_T nonChannelDimsProductIdx;
  int32_T v1;
  int32_T vk;
  real32_T dataExp[10];
  real32_T f;
  real32_T sumX;
  boolean_T emlrtHadParallelError = false;
  boolean_T exitg1;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  iComputeSoftmaxForCpu_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(iComputeSoftmaxForCpu_numThreads) private(    \
        sumX, dataExp, nonChannelSubscriptIndices, emlrtJBEnviron, b_st, vk,   \
            v1, i, idx, k, f, r, r1, exitg1)                                   \
    firstprivate(st, emlrtHadParallelError)
  {
    if (setjmp(emlrtJBEnviron) == 0) {
      st.prev = sp;
      st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
      st.site = NULL;
      emlrtSetJmpBuf(&st, &emlrtJBEnviron);
      b_st.prev = &st;
      b_st.tls = st.tls;
    } else {
      emlrtHadParallelError = true;
    }
#pragma omp for nowait
    for (nonChannelDimsProductIdx = 0; nonChannelDimsProductIdx < 40;
         nonChannelDimsProductIdx++) {
      if (emlrtHadParallelError) {
        continue;
      }
      if (setjmp(emlrtJBEnviron) == 0) {
        vk = nonChannelDimsProductIdx / 10;
        v1 = nonChannelDimsProductIdx - vk * 10;
        if ((v1 + 1 < 1) || (v1 + 1 > 10)) {
          emlrtDynamicBoundsCheckR2012b(v1 + 1, 1, 10, &emlrtBCI, &st);
        }
        i = 10 * v1 + 100 * vk;
        sumX = xdata[i];
        if (!muSingleScalarIsNaN(sumX)) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 11)) {
            if (!muSingleScalarIsNaN(xdata[((k + 10 * v1) + 100 * vk) - 1])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }
        if (idx != 0) {
          sumX = xdata[((idx + 10 * v1) + 100 * vk) - 1];
          idx++;
          for (k = idx; k < 11; k++) {
            f = xdata[((k + 10 * v1) + 100 * vk) - 1];
            if (sumX < f) {
              sumX = f;
            }
          }
        }
        r = _mm_set1_ps(sumX);
        _mm_storeu_ps(&dataExp[0], _mm_sub_ps(_mm_loadu_ps(&xdata[i]), r));
        _mm_storeu_ps(&dataExp[4], _mm_sub_ps(_mm_loadu_ps(&xdata[i + 4]), r));
        dataExp[8] = xdata[i + 8] - sumX;
        dataExp[9] = xdata[i + 9] - sumX;
        b_st.site = &xb_emlrtRSI;
        lambdaForColumnMajorGeneric(&b_st, dataExp);
        sumX = b_sumColumnB(dataExp);
        if ((v1 + 1 < 1) || (v1 + 1 > 10)) {
          emlrtDynamicBoundsCheckR2012b(v1 + 1, 1, 10, &b_emlrtBCI, &st);
        }
        r = _mm_loadu_ps(&dataExp[0]);
        r1 = _mm_set1_ps(sumX);
        _mm_storeu_ps(&ydata[i], _mm_div_ps(r, r1));
        r = _mm_loadu_ps(&dataExp[4]);
        _mm_storeu_ps(&ydata[i + 4], _mm_div_ps(r, r1));
        ydata[i + 8] = dataExp[8] / sumX;
        ydata[i + 9] = dataExp[9] / sumX;
      } else {
        emlrtHadParallelError = true;
      }
    }
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

/* End of code generation (internal_softmax.c) */

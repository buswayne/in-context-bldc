/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * binaryElementwise.c
 *
 * Code generation for function 'binaryElementwise'
 *
 */

/* Include files */
#include "binaryElementwise.h"
#include "rt_nonfinite.h"
#include "test_H10_40k_data.h"
#include "omp.h"
#include <xmmintrin.h>

/* Function Declarations */
static void elementwise_addition(const emlrtStack *sp,
                                 const real32_T *inputTensor0,
                                 const real32_T *inputTensor1,
                                 real32_T *outputTensor);

/* Function Definitions */
static void elementwise_addition(const emlrtStack *sp,
                                 const real32_T *inputTensor0,
                                 const real32_T *inputTensor1,
                                 real32_T *outputTensor)
{
  __m128 regSimd4_0;
  __m128 regSimd4_1;
  jmp_buf emlrtJBEnviron;
  jmp_buf *volatile emlrtJBStack;
  emlrtStack st;
  int32_T elementwise_addition_numThreads;
  int32_T regSimd4_0_tmp;
  int32_T simdBlockIdx;
  boolean_T emlrtHadParallelError = false;
  emlrtEnterParallelRegion((emlrtCTX)sp, omp_in_parallel());
  emlrtPushJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  elementwise_addition_numThreads = emlrtAllocRegionTLSs(
      sp->tls, omp_in_parallel(), omp_get_max_threads(), omp_get_num_procs());
#pragma omp parallel num_threads(elementwise_addition_numThreads) private(     \
        st, emlrtJBEnviron, regSimd4_0_tmp, regSimd4_0, regSimd4_1)            \
    firstprivate(emlrtHadParallelError)
  {
    if (setjmp(emlrtJBEnviron) == 0) {
      st.prev = sp;
      st.tls = emlrtAllocTLS((emlrtCTX)sp, omp_get_thread_num());
      st.site = NULL;
      emlrtSetJmpBuf(&st, &emlrtJBEnviron);
    } else {
      emlrtHadParallelError = true;
    }
#pragma omp for nowait
    for (simdBlockIdx = 0; simdBlockIdx < 40; simdBlockIdx++) {
      if (emlrtHadParallelError) {
        continue;
      }
      if (setjmp(emlrtJBEnviron) == 0) {
        regSimd4_0_tmp = simdBlockIdx << 2;
        regSimd4_0 = _mm_loadu_ps(&inputTensor0[regSimd4_0_tmp]);
        regSimd4_1 = _mm_loadu_ps(&inputTensor1[regSimd4_0_tmp]);
        regSimd4_0 = _mm_add_ps(regSimd4_1, regSimd4_0);
        _mm_storeu_ps(&outputTensor[regSimd4_0_tmp], regSimd4_0);
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      } else {
        emlrtHadParallelError = true;
      }
    }
  }
  emlrtPopJmpBuf((emlrtCTX)sp, &emlrtJBStack);
  emlrtExitParallelRegion((emlrtCTX)sp, omp_in_parallel());
}

void binaryElementwise(const emlrtStack *sp, const real32_T input1[160],
                       const real32_T input2[160], real32_T Z[160])
{
  elementwise_addition(sp, &input1[0], &input2[0], &Z[0]);
}

/* End of code generation (binaryElementwise.c) */
